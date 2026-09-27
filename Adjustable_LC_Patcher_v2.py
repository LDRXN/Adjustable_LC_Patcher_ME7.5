#!/usr/bin/env python3
"""ME7.5 adjustable LC, portable-by-detection, Python 3.10+, standard library.

Quick start (replace paths with your own, matching files):
    python Adjustable_LC_Patcher.py input.bin --ecu input.ecu --defaults --dry-run
    python Adjustable_LC_Patcher.py input.bin --ecu input.ecu --defaults --rpm 3500
    python Adjustable_LC_Patcher.py --help

Without --defaults or explicit parameters, a terminal displays a simple menu.
Build outputs: a NEW *_ADJUSTABLE_LC_BENCH.bin, .log and .json. An input BIN
is never overwritten. With no checksum executable, the output is uncorrected.

Driver inputs, RPM, vehicle speed, ignition outputs and MIL come from the
matching ME7Info .ecu, including widths, masks and conversion factors.
Firmware hooks are identified by complete supported instruction structures;
they are NOT addresses from an HN/DR address table. Unknown variants abort.

Implements the adjustable LC part of the supplied User Adjustable code.pdf:
two brake releases, three clutch releases, blinking indication, brake to
capture RPM; low-speed clutch LC with lower/upper ignition zones and cut.
The PDF's unrelated MapSwitch and NLS blocks are not installed.

Important integration change: intercept the OEM output-disable command
read instead of reserving an undocumented bit in the OEM request mask.
OEM disable requests retain priority. See --help and generated build log.
The reference B_kr branch and both MIL output paths are included, using
addresses resolved from the matching .ecu and correlated OEM instructions.
The default brake input is the OEM combined brake condition used by the
reference; --brake-symbol retains an explicit named-signal override.
This remains a new integration of LC, not a byte-for-byte copy. Hardware
equivalence of the output-disable command overlay is unverified.

This is a BENCH build. A symbol-gap RAM allocation is a candidate, not proof
against indirect accesses. Prefer an explicitly reserved LC_RAM measurement
in .ecu (size >=32, mask 0), or --ram-base after reviewing the RAM candidate.
No claim of compatibility with every ME7.5 software or on-car validation.

Instruction reference: https://www.keil.com/dd/docs/datashts/infineon/c166ism.pdf
Original reference: https://nefariousmotorsports.com/forum/index.php?topic=19193.0
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, asdict
from decimal import Decimal, ROUND_HALF_UP
from pathlib import Path

VERSION = '0.2-bench'
FLASH_BASE = 0x800000
SIZE = 0x100000
MARKER = b'ADJLC_DYNAMIC_V1\0'
RAM_SIZE = 32
# Private state offsets, not ECU-specific addresses.
MAGIC, INVERSE, TARGET, SEQUENCE, TIMER, CAPTURE, PHASE, BLINK, DIVIDER = range(0, 18, 2)
UC, EQ, NE, ULT, UGE, UGT, ULE = 0, 2, 3, 8, 9, 14, 15


class PatchError(RuntimeError):
    pass


def require(condition, message):
    if not condition:
        raise PatchError(message)


def u16(value):
    require(0 <= value <= 0xFFFF, f'16-bit value out of range: {value}')
    return value.to_bytes(2, 'little')


def align(value, boundary=16):
    return (value + boundary - 1) & -boundary


def far_call(offset):
    require(0 <= offset < SIZE and offset % 2 == 0, 'Invalid CALLS destination')
    addr = FLASH_BASE + offset
    return bytes((0xDA, addr >> 16)) + u16(addr & 0xFFFF)


def far_jump(offset):
    return b'\xFA' + far_call(offset)[1:]


@dataclass(frozen=True)
class Signal:
    name: str
    alias: str
    address: int
    size: int
    mask: int
    unit: str
    signed: int
    inverse: int
    factor: str
    offset: str

    def raw(self, physical, *, signed=False):
        require(self.inverse == 0, f'{self.name}: inverse conversion unsupported')
        a, b = Decimal(self.factor), Decimal(self.offset)
        require(a.is_finite() and b.is_finite() and a > 0,
                f'{self.name}: invalid conversion')
        value = (Decimal(str(physical)) + b) / a  # ME7Logger phys=A*raw-B
        raw = int(value.to_integral_value(rounding=ROUND_HALF_UP))
        bits = self.size * 8
        low, high = (-(1 << (bits - 1)), (1 << (bits - 1)) - 1) if signed else (0, (1 << bits) - 1)
        require(low <= raw <= high, f'{self.name}: {physical} is outside signal range')
        return raw & ((1 << bits) - 1)

    def physical(self, raw):
        if self.signed and raw & (1 << (8 * self.size - 1)):
            raw -= 1 << (8 * self.size)
        return float(Decimal(self.factor) * raw - Decimal(self.offset))


def read_ecu(path):
    text = path.read_text(encoding='latin-1')
    entries, identity = {}, {}
    section = ''
    for lineno, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith((';', '#')):
            continue
        if line.startswith('[') and line.endswith(']'):
            section = line[1:-1].casefold()
            continue
        if section == 'identification' and '=' in line:
            key, value = line.split('=', 1)
            identity[key.strip().casefold()] = value.split(';', 1)[0].strip().strip('{}').strip()
        if section != 'measurements':
            continue
        parts = []
        def protect(m):
            parts.append(m.group(1))
            return f'@@{len(parts)-1}@@'
        protected = re.sub(r'\{([^}]*)\}', protect, line)
        cols = next(csv.reader([protected], skipinitialspace=True))
        cols = [c.strip() for c in cols]
        cols = [parts[int(c[2:-2])] if re.fullmatch(r'@@\d+@@', c) else c for c in cols]
        if len(cols) < 10:
            continue
        try:
            sig = Signal(cols[0], cols[1], int(cols[2], 16), int(cols[3]),
                         int(cols[4], 16), cols[5], int(cols[6]), int(cols[7]),
                         str(Decimal(cols[8])), str(Decimal(cols[9])))
        except (ValueError, ArithmeticError) as exc:
            raise PatchError(f'.ecu line {lineno}: invalid measurement') from exc
        key = sig.name.casefold()
        entries.setdefault(key, []).append(sig)
    require(entries, 'No [Measurements] in .ecu')
    return entries, identity


def resolve(entries, names, *, alias=None, bit=False, sizes=(1, 2)):
    found = []
    for name in names:
        if name.casefold() in entries:
            found = entries[name.casefold()]
            break
    if not found and alias:
        found = [s for group in entries.values() for s in group if s.alias.casefold() == alias.casefold()]
    require(found, f'Missing .ecu signal: {" / ".join(names)}')
    keys = {(s.address, s.size, s.mask, s.signed, s.inverse, s.factor, s.offset) for s in found}
    require(len(keys) == 1, f'Ambiguous .ecu signal {names[0]}')
    s = found[0]
    require(s.size in sizes and 0 <= s.address < FLASH_BASE and
            (s.address & 0xFFFF) + s.size <= 0x10000,
            f'{s.name}: unsupported memory address/width')
    if s.size == 2:
        require(s.address % 2 == 0, f'{s.name}: unaligned WORD')
    if bit:
        require(s.mask > 0 and s.mask < 1 << (8 * s.size) and s.mask & (s.mask - 1) == 0,
                f'{s.name}: a single-bit mask is required')
        require(s.inverse==0 and s.signed==0 and Decimal(s.factor)==1 and Decimal(s.offset)==0,
                f'{s.name}: unsupported bit polarity/conversion')
    else:
        require(s.mask == 0, f'{s.name}: scalar signal must have mask 0')
        require(s.inverse == 0 and Decimal(s.factor).is_finite() and Decimal(s.factor) > 0
                and Decimal(s.offset).is_finite(), f'{s.name}: unsupported conversion')
    return s


def signals_from_ecu(entries, brake_name='B_br'):
    require(not any(re.fullmatch(r'zwoutar_\d+',key) and int(key.rsplit('_',1)[1])>=4
                    for key in entries), 'Only the four-cylinder ignition array is supported')
    s = {
        'rpm': resolve(entries, ['nmot_w'], sizes=(2,)),
        'rpm_coarse': resolve(entries, ['nmot'], sizes=(1,)),
        'speed': resolve(entries, ['vfzg', 'vfzg_w', 'vfzroh_w']),
        'clutch': resolve(entries, ['B_kuppl'], alias='CondClutchPedalSwitch', bit=True),
        'brake': resolve(entries, [brake_name], bit=True),
        'mil': resolve(entries, ['B_mil'], bit=True),
        'knock': resolve(entries, ['B_kr'], bit=True),
        'clock': resolve(entries, ['tnst_w'], alias='TimeSinceStart', sizes=(2,)),
        'angle': resolve(entries, ['zwout'], sizes=(1,)),
    }
    for i in range(4):
        s[f'angle{i}'] = resolve(entries, [f'zwoutar_{i}'], sizes=(1,))
    require(not s['rpm'].signed and not s['speed'].signed, 'RPM and speed must be unsigned')
    require(not s['clock'].signed and s['clock'].unit.casefold() in ('s','ms')
            and Decimal(s['clock'].offset)==0, 'Unsupported tnst_w time format')
    for key in ['angle'] + [f'angle{i}' for i in range(4)]:
        a = s[key]
        require(a.signed == 1 and (a.factor, a.offset) == (s['angle'].factor, s['angle'].offset),
                'Ignition outputs must share the same signed BYTE conversion')
    addrs = [s[f'angle{i}'].address for i in range(4)]
    require(addrs == list(range(addrs[0], addrs[0] + 4)), 'Unsupported ignition array layout')
    require((s['brake'].address, s['brake'].mask) != (s['clutch'].address, s['clutch'].mask),
            'Brake and clutch resolve to the same bit')
    return s


def check_identity(binary, identity):
    for key in ('hwnumber', 'swnumber', 'partnumber', 'swversion'):
        require(identity.get(key), f'.ecu is missing identification {key}')
    hw,sw=identity['hwnumber'],identity['swnumber']
    require(re.fullmatch(r'\d{10}',hw) and re.fullmatch(r'\d{10}',sw),
            'Unsupported Bosch HW/SW identification format')
    # Calibration data may coincidentally contain an ASCII digit sequence.
    # Require the complete adjacent Bosch HW/SW record, not two loose hits.
    require((hw+sw).encode('ascii') in binary, '.ecu / BIN Bosch HW/SW identity mismatch')
    pn, version = identity['partnumber'].encode('ascii'), identity['swversion'].encode('ascii')
    positions = [m.start() for m in re.finditer(re.escape(pn), binary)]
    require(any(binary[p + len(pn):p + len(pn) + 64].split(b'\0',1)[0].rstrip().endswith(b' '+version)
                for p in positions),
            '.ecu / BIN SWVersion mismatch near identification record')


def matches(binary, pattern):
    return [m for m in re.finditer(pattern, binary, re.S) if m.start() % 2 == 0]


def unique(items, name):
    require(len(items) == 1, f'{name}: expected one supported match, found {len(items)}')
    return items[0]


def detect_dpp(binary):
    # Full startup sequence. A boot-time DPP0=0 is permitted; runtime DPP0
    # must point into the flash image, not into the internal ROM alias.
    pattern = rb'\xE6\x00(..)\xE6\x01(..)\xE6\x02(..)\xE6\x03(..)'
    candidates = set()
    for m in matches(binary, pattern):
        values = tuple(int.from_bytes(g, 'little') for g in m.groups())
        if values[0] * 0x4000 >= FLASH_BASE and values[3] == 3 and all(x < 1024 for x in values):
            candidates.add(values)
    return unique(sorted(candidates), 'runtime DPP initialization')


def near_address(physical, dpp):
    values = [(i << 14) | (physical & 0x3FFF) for i, page in enumerate(dpp)
              if page == physical >> 14]
    return unique(values, f'near addressing for 0x{physical:06X}')


@dataclass(frozen=True)
class Hook:
    role: str
    offset: int
    original: bytes
    context_start: int
    context: bytes
    kind: str = 'call'


def hook_patch(hook, destination):
    require(hook.kind in ('call', 'jump'), 'Unknown hook transfer type')
    return far_call(destination) if hook.kind == 'call' else far_jump(destination)


def bit_operand(signal):
    a, mask = signal.address, signal.mask
    if signal.size == 1 and a & 1:
        a -= 1
        mask <<= 8
    bit = mask.bit_length() - 1
    if 0xFD00 <= a <= 0xFDFE and a % 2 == 0:
        return (a - 0xFD00) // 2, bit
    require(False, f'{signal.name}: supported MIL hook needs a bit in FD00..FDFE')


def resolve_reference_brake(binary, entries, sig, dpp):
    """Trace the OEM combined brake producer, anchored in .ecu inputs.

    The HN reference tests the result of B_br OR B_bl OR the producer's
    fallback flag, not B_br alone. Neither HN nor DR .ecu names this result.
    The producer ties its address to named inputs; no FDxx offset is assumed.
    """
    pedal = sig['brake']
    light = resolve(entries, ['B_bl'], bit=True)
    cycle = resolve(entries, ['Z_brems'], bit=True, sizes=(2,))
    pq,pb = bit_operand(pedal); lq,lb = bit_operand(light)
    pattern = (re.escape(bytes((0x8A,pq,2,pb<<4,0x9A,lq,2,lb<<4))) +
               rb'\xE0\x1C\x0D\x04\xF2\xF4' +
               re.escape(u16(near_address(cycle.address,dpp))) +
               rb'\x68\x41\xF0\xC4\x48\xC0\x3A\x88(.)[\x30-\x3F]')
    m = unique(matches(binary,pattern), 'OEM combined brake producer')
    q,bit = m.group(1)[0],m.group()[-1]&15
    require(q<0x80, 'Combined brake condition must be internal bit-addressable RAM')
    combined = Signal('OEM_combined_brake','BIN producer anchored in B_br/B_bl/Z_brems',
                      0xFD00+2*q,2,1<<bit,'',0,0,'1','0')
    require((combined.address,combined.mask)!=(sig['clutch'].address,sig['clutch'].mask),
            'Combined brake condition overlaps clutch')
    sig.update(brake=combined,brake_pedal=pedal,brake_light=light,brake_cycle=cycle)
    return {'producer':m.start(),'context':m.group().hex(' '),
            'source':'BIN producer correlated with .ecu B_br, B_bl and Z_brems',
            'logic':'B_br OR B_bl OR (word containing Z_brems AND 1); OEM computed output'}


def reject_existing_reference(binary):
    # Distinctive LC decision sequence from the supplied reference. Addresses
    # remain wildcarded, so a relocated copy is also rejected.
    pattern = (rb'\x9A.\x63.\xD7\x00..\xF3\xF8..\xD7\x00..\x43\xF8..'
               rb'\x9D\x2F\xF2\xF5..\xD7\x00..\xF2\xF4..\x40\x54\x9D\x0B')
    require(not matches(binary,pattern),
            'Existing reference Adjustable LC detected. Use a BIN before LC installation; patches cannot be stacked.')


def detect_hooks(binary, sig, dpp):
    hooks = []
    # tnst_w increments by exactly one unit per running-engine call. Its
    # .ecu conversion therefore supplies the modeled task period. The hook
    # precedes the saturation test: it keeps running after tnst_w reaches
    # FFFF, unlike using tnst_w itself as a free-running clock.
    clock = re.escape(u16(near_address(sig['clock'].address, dpp)))
    tickpat = (rb'\xDB\x00\xF2\xF4' + clock +
               rb'\x46\xF4\xFF\xFF\x9D\x05\x9A.\x03.\x08\x41\xF6\xF4' + clock +
               rb'\x8A.\x02.\x9A.\x22.\xE6\xFC..\xE6\xFD..')
    m = unique(matches(binary, tickpat), 'tnst_w periodic increment hook')
    p = m.start() + 2
    hooks.append(Hook('tick', p, binary[p:p + 4], m.start(), m.group()))

    # Final pointer setup, after calculation of the four ignition values.
    # Includes the end of the Bosch routine to exclude its earlier setup.
    arr = re.escape(u16(near_address(sig['angle0'].address, dpp)))
    anglepat = (rb'\xE6\xFC' + arr +
                rb'\xF3\xF8..\x3D\x02\xB4\x0C..\xA9\x8C\xB1\x80\xF7\xF8..\x98\x60\xDB\x00')
    m = unique(matches(binary, anglepat), 'LC ignition output hook')
    p = m.start()
    # Bosch registers zwout through an output-pointer table, rather than a
    # nearby direct MOVB store. Correlate that startup registration as well.
    scalar = re.escape(u16(near_address(sig['angle'].address, dpp)))
    scalarpat = (rb'\xE6\xF4' + scalar +
                 rb'\xF6\xF4..\xE6\xF5..\xF6\xF5..\xA9\xA4\xC0\xA5\xF6\xF5..')
    unique(matches(binary, scalarpat), 'zwout output-pointer registration')
    hooks.append(Hook('angle', p, binary[p:p + 4], m.start(), m.group()))

    # Identify the factory disable-mask helpers and correlate their command
    # byte with the full command-consumer sequence. No F7C2/F7C3 address
    # or flash hook offset is hardcoded.
    helperpat = (rb'\xF0\x4C\xC0\x84\xE0\x15\x4C\x54\x75\xFA(..)'
                 rb'\xF3\xF8\1\x2D\x04\xE7\xFA\x9A\x00\xF7\xFA(..)\xDB\x00'
                 rb'\xF0\x4C\xC0\x84\xE0\x15\x4C\x54\xB1\xA0\x65\xFA\1'
                 rb'\xF3\xF8\1\x3D\x04\xE7\xFA\x65\x00\xF7\xFA\2\xDB\x00')
    helper = unique(matches(binary, helperpat), 'OEM output disable-mask helpers')
    command = helper.group(2)
    cutpat = (rb'\x88\x90\xDA...\xF3\xF8' + re.escape(command) +
              rb'\x47\xF8\x9A\x00\x3D\x05\xE6\xF4\xFF\x00\xF6\xF4(..)'
              rb'\x0D\x02\xF6\x8E\1')
    m = unique(matches(binary, cutpat), 'OEM output disable-command consumer')
    p = m.start() + 6
    hooks.append(Hook('cut', p, binary[p:p + 4], m.start(), m.group()))

    # Bit packing to the factory active-low MIL output. Preserve the OEM
    # indication; the LC signal can add illumination, never suppress it.
    src, bit = bit_operand(sig['mil'])
    mils = []
    for p in range(0x20000, len(binary)-8, 2):
        if (binary[p] == 0x3A and binary[p+1] == src and binary[p+3] >> 4 == bit
                and binary[p-4] in (0x3A, 0x4A) and binary[p+4] in (0x3A, 0x4A)
                and binary[p+2] < 0x80):
            mils.append(p)
    p = unique(mils, 'MIL output packing hook')
    hooks.append(Hook('mil', p, binary[p:p+4], p-8, binary[p-8:p+8]))

    # The PDF gate bypasses BSET B_kr during the cut phase, reaching the
    # existing BCLR instead. Replace BSET+JMPR with JMPS, not CALLS: returning
    # to hook+4 would execute BCLR even on the stock path.
    kq,kb = bit_operand(sig['knock'])
    coarse = re.escape(u16(near_address(sig['rpm_coarse'].address,dpp)))
    gatepat = (rb'\xC2\xF4..\x66\xF4\x10\x00\x2D\x02' +
               re.escape(bytes((kb<<4|15,kq,0x0D,1,kb<<4|14,kq,0x9A,kq,9,kb<<4))) +
               rb'\xF3\xF8' + coarse + rb'\xD7\x40(..)\x43\xF8(..)\xFD\x02')
    gate = unique(matches(binary,gatepat), 'B_kr LC branch')
    page,operand=(int.from_bytes(g,'little') for g in gate.groups())
    require(FLASH_BASE <= (page<<14)+(operand&0x3FFF) < FLASH_BASE+len(binary),
            'B_kr threshold is outside flash')
    p=gate.start()+10
    hooks.append(Hook('knock',p,binary[p:p+4],gate.start(),gate.group(),'jump'))

    # The second indication path constructs a status byte with B_mil in bit
    # 3. HN builds it on the R0 software stack; DR builds it in RL7. Correlate
    # the B_mil test and subsequent output-byte stores before accepting either.
    source_test = bytes((0x9A,src))
    tail = rb'(..)\xF3\xF8(..)\xF7\xF8(..)\xF2\xF4\2\x7C\x84\xF7\xF8(..)'
    stackpat = (re.escape(source_test+bytes((4,bit<<4))) +
                rb'\xA9\x80\x77\xF8\x08\x00\xB9\x80'
                rb'\x9A.\x04.\xA9\x80\x77\xF8\x20\x00\xB9\x80'
                rb'\x9A.\x04.\xA9\x80\x77\xF8\x80\x00\xB9\x80\xB4\x00' + tail)
    regpat = (rb'\xE1\x0E\x9A.\x01.\xE1\x4E' +
              re.escape(source_test+bytes((2,bit<<4))) +
              rb'\x77\xFE\x08\x00\x9A.\x02.\x77\xFE\x20\x00\xF7\xFE' + tail)
    packed = [(m,'stack') for m in matches(binary,stackpat)]
    packed += [(m,'register') for m in matches(binary,regpat)]
    status,variant=unique(packed,'MIL status-byte producer')
    p=status.start(1)-2
    operand=int.from_bytes(status.group(1),'little')
    physical=(dpp[operand>>14]<<14)|(operand&0x3FFF)
    require(0xF000 <= physical < 0xFE00,'Unsupported MIL status-byte destination')
    sig['mil_status']=Signal('OEM_MIL_status','BIN producer anchored in B_mil',
                             physical,1,0,'',0,0,'1','0')
    hooks.append(Hook('mil_status',p,binary[p:p+4],status.start(),status.group()))
    require(len({h.offset for h in hooks}) == len(hooks), 'Overlapping hooks')
    # A branch into bytes 2..3 would be invalid after replacement by CALLS.
    for h in hooks:
        for p in range(max(0,h.offset-256), min(len(binary)-4,h.offset+256),2):
            op = binary[p]
            target = None
            if op & 15 == 13:
                target = p+2+int.from_bytes(binary[p+1:p+2],'little',signed=True)*2
            elif op in (0x8A,0x9A,0xAA,0xBA):
                target = p+4+int.from_bytes(binary[p+2:p+3],'little',signed=True)*2
            if target == h.offset+2:
                raise PatchError(f'{h.role}: possible branch into overwritten instruction @0x{p:06X}')
    details = {'dpp': list(dpp), 'disable_helpers': helper.start(),
               'disable_mask_operand': int.from_bytes(helper.group(1),'little'),
               'disable_command_operand': int.from_bytes(command,'little'),
               'disable_output_mask_operand': int.from_bytes(m.group(1),'little'),
               'knock_gate':gate.start()+10,'mil_status_variant':variant,
               'mil_status_address':physical,'mil_status_mask':8}
    return hooks, details


def ram_references(binary, dpp):
    """Recognized direct/default-DPP, EXTS/EXTP and far-pointer references.

    This is a rejection filter, not a proof of unused RAM. Register-computed
    pointers, changes of DPP and access sizes of referenced arrays need review.
    In particular, a flash operand with the same low 14 bits is not a RAM use.
    """
    used = set()
    memory = {0x02,0x03,0x04,0x05,0x12,0x13,0x14,0x15,
              0x22,0x23,0x24,0x25,0x32,0x33,0x34,0x35,0x42,0x43,
              0x52,0x53,0x54,0x55,0x62,0x63,0x64,0x65,
              0x72,0x73,0x74,0x75,0x82,0x92,0xA2,0xB2,
              0x84,0x94,0xA4,0xB4,0xC2,0xC5,0xD2,0xD5,0xD6,
              0xF2,0xF3,0xF6,0xF7}
    def mark(addr):
        used.update((addr,addr+1))
    def default_address(word):
        return (dpp[word>>14]<<14)|(word&0x3FFF)
    def size(op):
        return 4 if ((op<0x80 and (2<=op%16<=7 or op%16==10)) or
                     op in memory or op in {0x86,0x96,0x87,0x97,0xA5,0xB5,0xA6,0xB6,
                     0xA7,0xB7,0x8A,0x9A,0xAA,0xBA,0xC4,0xC6,0xD4,0xD7,0xCA,0xDA,
                     0xE2,0xE4,0xE6,0xE7,0xF4,0xEA,0xFA}) else 2
    for p in range(0,len(binary)-7,2):
        op, reg = binary[p:p+2]
        word = int.from_bytes(binary[p+2:p+4],'little')
        if op in memory or (op==0xE6 and reg>=0xF0):
            mark(default_address(word))
        # Address/DPP pairs in adjacent immediate register loads.
        if op==0xE6 and reg>=0xF0 and binary[p+4]==0xE6 and binary[p+5]>=0xF0:
            page=int.from_bytes(binary[p+6:p+8],'little')
            if page<1024:
                mark((page<<14)|(word&0x3FFF))
        if op==0xD7 and reg & 0x0F == 0:
            kind=reg & 0xC0
            if kind not in (0,0x40):continue
            if word >= (256 if kind==0 else 1024):continue
            q=p+4
            for _ in range(((reg>>4)&3)+1):
                if q+4>len(binary):break
                if binary[q] in memory:
                    offset=int.from_bytes(binary[q+2:q+4],'little')
                    mark((word<<16)|offset if kind==0 else (word<<14)|(offset&0x3FFF))
                q+=size(binary[q])
    return used


def ram_candidate(binary, entries, dpp, explicit=None):
    allsig = [s for group in entries.values() for s in group]
    spans = [(s.address, s.address+s.size) for s in allsig]
    external = [s for s in allsig if 0x10000 <= s.address < FLASH_BASE]
    require(external, '.ecu has no external RAM measurements')
    segments = {s.address >> 16 for s in external}
    reserved = entries.get('lc_ram', [])
    if reserved and (explicit is None or explicit == reserved[0].address):
        require(len({(s.address,s.size,s.mask) for s in reserved}) == 1,
                'Ambiguous LC_RAM reservation in .ecu')
        r = reserved[0]
        require(r.size >= RAM_SIZE and r.mask == 0, 'LC_RAM must reserve at least 32 bytes, mask 0')
        explicit = r.address
        spans = [(s.address,s.address+s.size) for s in allsig if s.name.casefold() != 'lc_ram']
        source = '.ecu LC_RAM reservation'
    else:
        source = 'explicit --ram-base' if explicit is not None else 'automatic symbol-gap/reference candidate'
    used = ram_references(binary,dpp)
    def acceptable(a):
        if a & 1 or a >> 16 not in segments or (a & 0xFFFF)+RAM_SIZE > 0x10000:
            return False
        if any(a < end and start < a+RAM_SIZE for start,end in spans):
            return False
        return not any(a+i in used for i in range(RAM_SIZE))
    if explicit is not None:
        require(acceptable(explicit), 'Requested RAM overlaps a symbol/reference or is outside supported RAM')
        return explicit, source
    candidates = []
    for seg in sorted(segments):
        ss = sorted((a,b) for a,b in spans if a >> 16 == seg)
        if not ss:
            continue
        merged=[]
        for a,b in ss:
            if merged and a <= merged[-1][1]:
                merged[-1]=(merged[-1][0],max(b,merged[-1][1]))
            else:
                merged.append((a,b))
        for (_, end),(start,_) in zip(merged,merged[1:]):
            for a in range(align(end+64),start-RAM_SIZE-64+1,16):
                if acceptable(a):
                    candidates.append((min(a-end,start-a-RAM_SIZE),a))
    require(candidates, 'No RAM candidate: provide a genuinely reserved LC_RAM area in .ecu')
    return max(candidates)[1], source


@dataclass(frozen=True)
class Config:
    rpm: float = 3500
    min_rpm: float = 1500
    max_rpm: float = 5500
    speed: float = 3.75
    lower_window: float = 100
    upper_window: float = 100
    lower_angle: float = -20.25
    upper_angle: float = -30
    sequence_ticks: int = 80
    blink_ticks: int = 3
    engine_min_rpm: float = 500

    def validate(self, sig):
        require(all(math.isfinite(float(x)) for x in asdict(self).values()), 'Parameters must be finite')
        require(1000 <= self.min_rpm <= self.rpm <= self.max_rpm <= 8000, 'Require 1000 <= min <= default <= max <= 8000 RPM')
        require(0 < self.speed <= 20, 'LC speed threshold must be >0 and <=20 km/h')
        require(0 < self.lower_window < self.min_rpm and 0 < self.upper_window <= 1000, 'Invalid RPM windows')
        require(-90 <= self.lower_angle <= 0 and -90 <= self.upper_angle <= 0, 'Ignition angle must be -90..0 degrees')
        require(0 < self.engine_min_rpm < self.min_rpm, 'Engine-running threshold must be below min RPM')
        require(1 <= self.sequence_ticks <= 0x7FFF and 1 <= self.blink_ticks <= 0x7FFF, 'Tick counts must be 1..32767')
        rpm, speed, ang = sig['rpm'],sig['speed'],sig['angle']
        delta = lambda v: int((Decimal(str(v))/Decimal(rpm.factor)).to_integral_value(rounding=ROUND_HALF_UP))
        raw = {'default':rpm.raw(self.rpm), 'minimum':rpm.raw(self.min_rpm),
               'maximum':rpm.raw(self.max_rpm), 'engine':rpm.raw(self.engine_min_rpm),
               'speed':speed.raw(self.speed), 'lower':delta(self.lower_window),
               'upper':delta(self.upper_window), 'angle_low':ang.raw(self.lower_angle,signed=True),
               'angle_high':ang.raw(self.upper_angle,signed=True),
               'timeout':self.sequence_ticks, 'blink':self.blink_ticks}
        # Expose stable 100 ms control ticks. Infer the OEM task period from
        # its verified +1 increment and the matching .ecu time conversion.
        period_ms=Decimal(sig['clock'].factor)*(1000 if sig['clock'].unit.casefold()=='s' else 1)
        subticks=Decimal(100)/period_ms
        require(subticks==int(subticks) and 1<=subticks<=1000,
                'tnst_w task period must divide 100 ms exactly')
        raw['task_subticks']=int(subticks)
        require(raw['speed'] > 0 and raw['lower'] > 0 and raw['upper'] > 0, 'Parameter rounds to zero')
        require(raw['minimum'] > raw['lower'] and raw['maximum']+raw['upper'] <= 0xFFFF, 'RPM calculation would overflow')
        return raw


class Assembler:
    """Small C166 emitter; all jumps stay in one 64 KiB code segment."""
    def __init__(self, base):
        self.base, self.code, self.labels, self.fixups = base, bytearray(), {}, []

    def emit(self, *parts):
        for x in parts:
            self.code.extend(bytes((x,)) if isinstance(x,int) else x)

    def label(self, name):
        require(name not in self.labels, f'Duplicate label {name}')
        self.labels[name] = len(self.code)

    def jump(self, cc, target):
        pos=len(self.code)
        self.emit(0xEA,cc<<4,b'\0\0')
        self.fixups.append((pos+2,target))

    def imm(self, reg, value):
        self.emit(0xE6,0xF0+reg,u16(value))

    def cmp(self, reg, value):
        self.emit(0x46,0xF0+reg,u16(value))

    def alu_imm(self, opcode, reg, value):
        self.emit(opcode,0xF0+reg,u16(value))

    def mem(self, opcode, register, address):
        self.emit(0xD7,0x00,u16(address>>16))  # EXTS #segment,#1
        self.emit(opcode,register,u16(address&0xFFFF))

    def load(self, reg, address, size=2):
        self.mem(0xF2 if size==2 else 0xC2,0xF0+reg,address)

    def store(self, address, reg=4, size=2):
        require(reg <= 7 or size==2, 'Byte store requires RL0..RL7')
        self.mem(0xF6 if size==2 else 0xF7,0xF0+reg if size==2 else 0xF0+reg*2,address)

    def setmem(self, address, value):
        self.imm(4,value)
        self.store(address)

    def signal(self, reg, sig):
        self.load(reg,sig.address,sig.size)

    def bit_test(self, sig, *, clear=None, set_=None):
        self.signal(4,sig)
        self.alu_imm(0x66,4,sig.mask)
        if clear is not None:
            self.jump(EQ,clear)
        if set_ is not None:
            self.jump(NE,set_)

    def save(self):
        self.emit(0xEC,0x88)  # PUSH PSW, before flag-changing instructions
        for reg in (4,5,6):
            self.emit(0xEC,0xF0+reg)

    def restore(self):
        for reg in (6,5,4):
            self.emit(0xFC,0xF0+reg)
        self.emit(0xFC,0x88)

    def rets(self):
        self.emit(0xDB,0x00)

    def finish(self):
        for pos,label in self.fixups:
            require(label in self.labels,f'Unknown label {label}')
            dest=self.base+self.labels[label]
            require(dest>>16 == self.base>>16, 'Code crosses a 64 KiB segment')
            self.code[pos:pos+2]=u16(dest&0xFFFF)
        require(len(self.code)%2==0,'Odd instruction alignment')
        return bytes(self.code)


def emit_valid(a, ram, raw, magic, fail):
    a.load(4,ram+MAGIC); a.cmp(4,magic); a.jump(NE,fail)
    a.load(4,ram+INVERSE); a.cmp(4,magic^0xFFFF); a.jump(NE,fail)
    a.load(5,ram+TARGET); a.cmp(5,raw['minimum']); a.jump(ULT,fail)
    a.cmp(5,raw['maximum']); a.jump(UGT,fail)


def emit_active(a, sig, raw, fail):
    a.bit_test(sig['clutch'],clear=fail)
    a.signal(4,sig['speed']); a.cmp(4,raw['speed']); a.jump(UGE,fail)


def emit_tick(a, sig, raw, ram, magic, original):
    a.label('tick'); a.save()
    emit_valid(a,ram,raw,magic,'tick_init')
    a.jump(UC,'tick_ready')
    a.label('tick_init')
    # Publish magic last so other wrappers ignore partially initialized RAM.
    a.setmem(ram+MAGIC,0)
    a.imm(4,0)
    for off in range(6,RAM_SIZE,2): a.store(ram+off)
    a.setmem(ram+TARGET,raw['default'])
    a.setmem(ram+INVERSE,magic^0xFFFF)
    a.setmem(ram+MAGIC,magic)
    a.label('tick_ready')
    a.signal(4,sig['rpm']); a.cmp(4,raw['engine']); a.jump(ULT,'tick_reset')
    a.signal(4,sig['speed']); a.cmp(4,raw['speed']); a.jump(UGE,'tick_reset')
    # All human-input timers use 100 ms ticks, independent of the task rate.
    a.load(4,ram+DIVIDER); a.alu_imm(0x06,4,1); a.cmp(4,raw['task_subticks'])
    a.jump(UGE,'tick_control')
    a.store(ram+DIVIDER); a.jump(UC,'tick_exit')
    a.label('tick_control'); a.setmem(ram+DIVIDER,0)
    a.load(4,ram+CAPTURE); a.cmp(4,0); a.jump(NE,'tick_capture')

    # Reference sequence: low seven bits=count, bit 7=pedal held.
    a.load(6,ram+SEQUENCE); a.cmp(6,0); a.jump(EQ,'seq_stage')
    a.load(4,ram+TIMER); a.alu_imm(0x06,4,1)
    a.cmp(4,raw['timeout']); a.jump(UGT,'tick_reset')
    a.store(ram+TIMER)
    a.label('seq_stage')
    a.emit(0xF0,0x56)  # MOV R5,R6
    a.alu_imm(0x66,5,0x7F); a.cmp(5,2); a.jump(UGE,'seq_clutch')
    a.bit_test(sig['clutch'],set_='tick_reset')
    a.bit_test(sig['brake'],clear='seq_release_brake')
    a.alu_imm(0x76,6,0x80); a.jump(UC,'seq_store')
    a.label('seq_release_brake')
    a.emit(0xF0,0x46); a.alu_imm(0x66,4,0x80); a.jump(EQ,'tick_exit')
    a.alu_imm(0x66,6,0x7F); a.alu_imm(0x06,6,1); a.jump(UC,'seq_store')

    a.label('seq_clutch')
    a.cmp(5,5); a.jump(UGE,'tick_reset')
    a.bit_test(sig['brake'],set_='tick_reset')
    a.bit_test(sig['clutch'],clear='seq_release_clutch')
    a.alu_imm(0x76,6,0x80); a.jump(UC,'seq_store')
    a.label('seq_release_clutch')
    a.emit(0xF0,0x46); a.alu_imm(0x66,4,0x80); a.jump(EQ,'tick_exit')
    a.alu_imm(0x66,6,0x7F); a.alu_imm(0x06,6,1)
    a.cmp(6,5); a.jump(NE,'seq_store')
    a.setmem(ram+CAPTURE,1); a.setmem(ram+PHASE,1)
    a.setmem(ram+BLINK,raw['blink'])
    a.setmem(ram+SEQUENCE,0); a.setmem(ram+TIMER,0)
    a.jump(UC,'tick_exit')
    a.label('seq_store'); a.store(ram+SEQUENCE,6); a.jump(UC,'tick_exit')

    a.label('tick_capture')
    a.bit_test(sig['brake'],set_='tick_confirm')
    a.load(4,ram+BLINK); a.cmp(4,1); a.jump(ULE,'blink_toggle')
    a.alu_imm(0x26,4,1); a.store(ram+BLINK); a.jump(UC,'tick_exit')
    a.label('blink_toggle')
    a.load(4,ram+PHASE); a.alu_imm(0x56,4,1); a.store(ram+PHASE)
    a.setmem(ram+BLINK,raw['blink']); a.jump(UC,'tick_exit')
    a.label('tick_confirm')
    a.signal(4,sig['rpm']); a.cmp(4,raw['minimum']); a.jump(ULT,'capture_default')
    a.cmp(4,raw['maximum']); a.jump(ULE,'capture_store')
    a.label('capture_default'); a.imm(4,raw['default'])
    a.label('capture_store'); a.store(ram+TARGET)
    a.label('tick_reset')
    a.imm(4,0)
    for off in (SEQUENCE,TIMER,CAPTURE,PHASE,BLINK,DIVIDER): a.store(ram+off)
    a.label('tick_exit'); a.restore(); a.emit(original); a.rets()


def emit_angle(a,sig,raw,ram,magic,original):
    a.label('angle'); a.save()
    emit_valid(a,ram,raw,magic,'angle_exit')
    emit_active(a,sig,raw,'angle_exit')
    a.signal(4,sig['rpm']); a.emit(0x40,0x45); a.jump(UGE,'angle_upper')
    a.alu_imm(0x26,5,raw['lower']); a.emit(0x40,0x45); a.jump(ULT,'angle_exit')
    a.imm(4,raw['angle_low']); a.jump(UC,'angle_write')
    a.label('angle_upper')
    a.alu_imm(0x06,5,raw['upper']); a.emit(0x40,0x45); a.jump(UGE,'angle_exit')
    a.imm(4,raw['angle_high'])
    a.label('angle_write'); a.cmp(4,0); a.jump(EQ,'angle_exit')
    for key in ['angle0','angle1','angle2','angle3','angle']:
        a.store(sig[key].address,4,1)
    a.label('angle_exit'); a.restore(); a.emit(original); a.rets()


def emit_cut_condition(a,sig,raw,ram,magic,fail):
    emit_valid(a,ram,raw,magic,fail)
    emit_active(a,sig,raw,fail)
    a.alu_imm(0x06,5,raw['upper'])
    a.signal(4,sig['rpm']); a.emit(0x40,0x45); a.jump(ULT,fail)


def emit_cut(a,sig,raw,ram,magic,original):
    a.label('cut'); a.save()
    emit_cut_condition(a,sig,raw,ram,magic,'cut_stock')
    a.restore(); a.emit(0xE7,0xF8,0x9A,0); a.rets()
    a.label('cut_stock'); a.restore(); a.emit(original); a.rets()


def emit_knock(a,sig,raw,ram,magic,hook):
    a.label('knock'); a.save()
    emit_cut_condition(a,sig,raw,ram,magic,'knock_stock')
    # Same two continuation choices as the reference: existing BCLR on LC
    # cut, BSET followed by skipping that BCLR otherwise. No extra return.
    a.restore(); a.emit(far_jump(hook.offset+4))
    a.label('knock_stock'); a.restore(); a.emit(hook.original[:2])
    a.emit(far_jump(hook.offset+6))


def emit_mil(a,sig,raw,ram,magic,original):
    a.label('mil'); a.emit(original); a.save()
    emit_valid(a,ram,raw,magic,'mil_exit')
    a.load(4,ram+CAPTURE); a.cmp(4,1); a.jump(NE,'mil_exit')
    a.load(4,ram+PHASE); a.cmp(4,1); a.jump(NE,'mil_exit')
    # Active-low output: the reference BMOVN maps source=1 to destination=0.
    # Restoring original PSW makes the wrapper transparent to following code.
    a.emit(((original[3]&15)<<4)|0x0E, original[2])
    a.label('mil_exit'); a.restore(); a.rets()


def emit_mil_status(a,sig,raw,ram,magic,original):
    # Replay the actual producer store first (B4 00 [R0] or F7 FE RL7).
    # Save its resulting PSW; restore all registers/flags after the overlay.
    a.label('mil_status'); a.emit(original); a.save()
    emit_valid(a,ram,raw,magic,'mil_status_exit')
    a.load(4,ram+CAPTURE); a.cmp(4,1); a.jump(NE,'mil_status_exit')
    a.load(4,ram+PHASE); a.cmp(4,1); a.jump(NE,'mil_status_exit')
    a.load(4,sig['mil_status'].address,1); a.alu_imm(0x76,4,8)
    a.store(sig['mil_status'].address,4,1)
    a.label('mil_status_exit'); a.restore(); a.rets()


def code_cave(binary, size):
    for start in (0xA0000,0x70000,0x20000):
        pos=start
        while pos+size <= len(binary):
            p=binary.find(b'\xFF'*size,pos)
            if p < 0: break
            p=align(p)
            if p//0x10000 != (p+size-1)//0x10000:
                pos=align(p,0x10000); continue
            if p+size<=len(binary) and binary[p:p+size]==b'\xFF'*size:
                return p
            pos=p+16
    raise PatchError(f'No aligned code cave of {size} bytes in one 64 KiB segment')


def generate(base,sig,raw,ram,hooks,magic):
    a=Assembler(base)
    originals={h.role:h.original for h in hooks}
    emit_tick(a,sig,raw,ram,magic,originals['tick'])
    emit_angle(a,sig,raw,ram,magic,originals['angle'])
    emit_cut(a,sig,raw,ram,magic,originals['cut'])
    emit_mil(a,sig,raw,ram,magic,originals['mil'])
    emit_knock(a,sig,raw,ram,magic,next(h for h in hooks if h.role=='knock'))
    emit_mil_status(a,sig,raw,ram,magic,originals['mil_status'])
    return a.finish(),{name:base+a.labels[name] for name in originals}


def build(binary,entries,identity,config,*,ram_base=None,brake_name=None):
    require(len(binary)==SIZE,'Expected a 1 MiB ME7.5 BIN')
    require(MARKER not in binary,'This BIN already contains the Adjustable LC patch')
    reject_existing_reference(binary)
    check_identity(binary,identity)
    sig=signals_from_ecu(entries,brake_name or 'B_br')
    dpp=detect_dpp(binary)
    brake_details=({'source':f'explicit .ecu signal {brake_name}'} if brake_name else
                   resolve_reference_brake(binary,entries,sig,dpp))
    raw=config.validate(sig)
    hooks,detected=detect_hooks(binary,sig,dpp)
    detected['brake']=brake_details
    ram,ram_source=ram_candidate(binary,entries,dpp,ram_base)
    seed=json.dumps({'version':VERSION,'raw':raw,'signals':{k:asdict(v) for k,v in sig.items()},'ram':ram},sort_keys=True).encode()
    magic=int.from_bytes(hashlib.sha256(seed).digest()[:2],'little') or 0x4C41
    provisional,_=generate(0,sig,raw,ram,hooks,magic)
    size=align(len(provisional))+len(MARKER)+16
    cave=code_cave(binary,size)
    code,wrappers=generate(cave,sig,raw,ram,hooks,magic)
    out=bytearray(binary)
    for h in hooks:
        require(binary[h.context_start:h.context_start+len(h.context)]==h.context,'Hook context changed')
        require(not (cave <= h.offset < cave+size),'Code cave overlaps hook')
    out[cave:cave+len(code)]=code
    metadata=cave+align(len(code))
    meta=MARKER+ram.to_bytes(4,'little')+u16(magic)+u16(len(code))+hashlib.sha256(seed).digest()[:8]
    out[metadata:metadata+len(meta)]=meta
    for h in hooks:
        out[h.offset:h.offset+4]=hook_patch(h,wrappers[h.role])
    require(len(out)==SIZE,'Output size changed')
    allowed=[(cave,cave+size)]+[(h.offset,h.offset+4) for h in hooks]
    require(all(any(a<=i<b for a,b in allowed) for i,(x,y) in enumerate(zip(binary,out)) if x!=y),
            'Unexpected bytes changed')
    report={'application':'Adjustable LC Patcher','version':VERSION,'status':'BENCH / CHECKSUM REQUIRED',
            'identity':identity,'input_sha256':hashlib.sha256(binary).hexdigest(),
            'output_sha256':hashlib.sha256(out).hexdigest(),
            'parameters':asdict(config),'raw_parameters':raw,
            'signals':{k:asdict(v) for k,v in sig.items()},'detected':detected,
            'ram':{'base':ram,'size':RAM_SIZE,'source':ram_source,'magic':magic,
                   'limitation':'RAM ownership is NOT verified: computed pointers and local DPP changes are not fully tracked.'},
            'code':{'start':cave,'length':len(code),'reserved':size,'metadata':metadata,'wrappers':wrappers},
            'hooks':[{'role':h.role,'offset':h.offset,'original':h.original.hex(' '),
                      'patched':hook_patch(h,wrappers[h.role]).hex(' '),'kind':h.kind,
                      'context_start':h.context_start,'context':h.context.hex(' ')} for h in hooks],
            'controls':'2 brake releases, 3 clutch releases, set RPM, brake to confirm',
            'timing':('sequence_ticks/blink_ticks use 100 ms control ticks; '
                      f"OEM period inferred from tnst_w conversion: {100/raw['task_subticks']:g} ms. "
                      'Default sequence window ~8 s and MIL half-period 300 ms; bench verification required.'),
            'integration':('OEM output-disable command consumer overlay; original OEM requests remain effective. '
                           'The same LC cut predicate redirects the reference B_kr branch. '
                           'Both active-low output and packed status-byte MIL paths are overlaid.'),
            'changes_from_pdf':['One configurable default RPM for initialization and invalid capture.',
                                'Moving above threshold or stopping engine clears the whole setup sequence.',
                                'State, vehicle signals and hook addresses relocated; registers/PSW preserved.',
                                'Both MIL overlays never clear a factory MIL request.',
                                'Input timers normalized to 100 ms ticks using the tnst_w increment task.',
                                'Standalone adjustable LC; PDF MapSwitch/NLS fragments not installed.'],
            'validation_limits':['Supported hook structures only, not all ME7.5 software.',
                                 'Cut command and B_kr evaluate the live LC predicate separately; OEM request bit 7 is not reserved.',
                                 'The B_kr hook reproduces one reference branch, not a guarantee that all knock paths are inhibited.',
                                 'The status-byte MIL producer is validated, but actual cluster/CAN routing needs ECU validation.',
                                 'No ECU, CAN, hardware-stack timing or vehicle validation by this program.']}
    return bytes(out),report


def make_text_report(report):
    rows=[f'Adjustable LC Patcher {VERSION}',report['status'],'',
          'Identity: '+str(report['identity']),
          'Controls: '+report['controls'],'','Signals (.ecu; OEM_* resolved from correlated BIN producers):']
    for role,s in report['signals'].items():
        rows.append(f"  {role:10} {s['name']:18} 0x{s['address']:06X} size={s['size']} mask=0x{s['mask']:04X} factor={s['factor']}")
    rows += ['', 'Hooks (file offsets):']
    for h in report['hooks']:
        dst=report['code']['wrappers'][h['role']]
        rows.append(f"  {h['role']:10} 0x{h['offset']:06X}: {h['original']} -> 0x{dst:06X} ({h['kind']})")
    rows += ['', 'Brake input: '+str(report['detected']['brake'])]
    c,r=report['code'],report['ram']
    rows += ['',f"Code: 0x{c['start']:06X}..0x{c['start']+c['length']-1:06X}",
             f"RAM: 0x{r['base']:06X}..0x{r['base']+r['size']-1:06X} ({r['source']})",
             r['limitation'],'','Parameters:']
    rows += [f'  {k}: {v}' for k,v in report['parameters'].items()]
    rows += ['','Raw encoded parameters: '+str(report['raw_parameters'])]
    rows += ['',report['timing'],'',report['integration'],'','Differences from PDF:']
    rows += ['  '+x for x in report['changes_from_pdf']]
    rows += ['', 'Validation limits:']+['  '+x for x in report['validation_limits']]
    rows += ['', 'Input SHA256: '+report['input_sha256'],'Output SHA256: '+report['output_sha256']]
    return '\n'.join(rows)+'\n'


def resolve_ecu_path(binary_path, explicit):
    if explicit:
        return explicit.expanduser().resolve()
    exact=binary_path.with_suffix('.ecu')
    if exact.is_file():return exact
    choices=list(binary_path.parent.glob('*.ecu'))
    require(len(choices)==1,'Specify the matching .ecu using --ecu FILE')
    return choices[0]


def output_path_for(inp,explicit):
    if explicit:
        out=explicit.expanduser().resolve()
        require(out!=inp and not out.exists(),'Output must be a new file, different from input')
        require(out.suffix.casefold()=='.bin','Output filename must end in .bin')
        require(not any(out.with_suffix(s).exists() for s in ('.log','.json')),'Output log already exists')
        return out
    suffix='_ADJUSTABLE_LC_BENCH'
    out=inp.with_name(inp.stem+suffix+'.bin'); i=2
    while any(out.with_suffix(s).exists() for s in ('.bin','.log','.json')):
        out=inp.with_name(inp.stem+suffix+f'_{i}.bin'); i+=1
    return out


def run_checksum(tool,raw,folder):
    require(tool.is_file(),'me7sum executable does not exist')
    with tempfile.TemporaryDirectory(prefix='adjustable_lc_',dir=folder) as td:
        src,dst=Path(td)/'raw.bin',Path(td)/'checked.bin'
        src.write_bytes(raw)
        cmd=[str(tool),str(src),str(dst)]
        if tool.suffix.casefold()=='.exe' and os.name!='nt':
            wine=shutil.which('wine')
            require(wine,'Windows me7sum.exe needs Wine on this OS')
            cmd.insert(0,wine)
        try:
            result=subprocess.run(cmd,capture_output=True,text=True,timeout=60)
        except (OSError,subprocess.TimeoutExpired) as exc:
            raise PatchError(f'me7sum failed: {exc}') from exc
        require(result.returncode==0 and dst.is_file(),'me7sum failed: '+result.stdout[-2000:]+result.stderr[-1000:])
        checked=dst.read_bytes()
        require(len(checked)==SIZE,'me7sum returned an invalid size')
        return checked,result.stdout+result.stderr


def parser():
    p=argparse.ArgumentParser(description='ME7.5 adjustable LC: dynamic .ecu signals and validated BIN hooks.')
    p.add_argument('bin_file',type=Path,nargs='?')
    p.add_argument('--ecu',type=Path,help='Matching ME7Info definition')
    p.add_argument('--output','-o',type=Path,help='New output BIN; input is never overwritten')
    p.add_argument('--dry-run',action='store_true',help='Analyze and generate in memory; no files written')
    p.add_argument('--defaults','--non-interactive',action='store_true',help='Use defaults for parameters not specified')
    for name,default in asdict(Config()).items():
        p.add_argument('--'+name.replace('_','-'),type=int if name in ('sequence_ticks','blink_ticks') else float,
                       default=None,help=f'Default: {default}'+(' (100 ms ticks)' if name in ('sequence_ticks','blink_ticks') else ''))
    p.add_argument('--brake-symbol',default=None,
                   help='Explicit .ecu brake signal; by default trace the reference combined brake producer')
    p.add_argument('--ram-base',type=lambda x:int(x,0),help='Explicit reserved RAM base; preferred source is .ecu LC_RAM')
    p.add_argument('--me7sum',type=Path,help='Optional me7sum executable; correction is performed last')
    return p


def main(argv=None):
    args=parser().parse_args(argv)
    print(f'\nME7.5 — Adjustable Launch Control — {VERSION}\n')
    if args.bin_file is None:
        require(sys.stdin.isatty(),'BIN path is required')
        args.bin_file=Path(input('BIN file: ').strip().strip('"'))
    inp=args.bin_file.expanduser().resolve()
    require(inp.is_file(),f'BIN not found: {inp}')
    ecu=resolve_ecu_path(inp,args.ecu)
    require(ecu.is_file(),f'.ecu not found: {ecu}')
    values=asdict(Config())
    supplied=any(getattr(args,k) is not None for k in values)
    if not args.defaults and not supplied and sys.stdin.isatty():
        print('  [1] Reference defaults: 3500 RPM, 1500–5500 RPM range, 3.75 km/h')
        print('  [2] Change default RPM and LC speed threshold')
        choice=input('Selection [1]: ').strip() or '1'
        require(choice in ('1','2'),'Select 1 or 2')
        if choice=='2':
            values['rpm']=float(input('Default LC RPM [3500]: ').strip() or '3500')
            values['speed']=float(input('LC speed threshold km/h [3.75]: ').strip() or '3.75')
    values.update({k:getattr(args,k) for k in values if getattr(args,k) is not None})
    entries,identity=read_ecu(ecu)
    output,report=build(inp.read_bytes(),entries,identity,Config(**values),ram_base=args.ram_base,brake_name=args.brake_symbol)
    report['input_file']=inp.name; report['ecu_file']=ecu.name
    print(make_text_report(report))
    if args.dry_run:
        print('ANALYSIS ONLY — no files written.')
        return 0
    out=output_path_for(inp,args.output)
    require(out.parent.is_dir(),'Output directory does not exist')
    tool=args.me7sum
    if tool is None:
        for folder in (Path(__file__).resolve().parent,inp.parent):
            for name in ('me7sum','me7sum.exe'):
                if (folder/name).is_file():tool=folder/name;break
            if tool:break
    if tool:
        corrected,checksum_log=run_checksum(tool.expanduser().resolve(),output,out.parent)
        # The checksum tool must not change any injected code or hook byte.
        c=report['code'];regions=[(c['start'],c['start']+c['reserved'])]+[(h['offset'],h['offset']+4) for h in report['hooks']]
        require(all(corrected[a:b]==output[a:b] for a,b in regions),'Checksum tool changed LC code/hooks')
        output=corrected
        report['checksum_tool_log']=checksum_log
        report['status']='BENCH / me7sum completed'
        report['output_sha256']=hashlib.sha256(output).hexdigest()
    # Exclusive creation prevents accidental overwrite, including races.
    created=[]
    try:
        for path,content in [(out,output),(out.with_suffix('.json'),json.dumps(report,indent=2,ensure_ascii=False).encode('utf-8')),
                             (out.with_suffix('.log'),make_text_report(report).encode('utf-8'))]:
            with path.open('xb') as f:
                created.append(path)
                f.write(content)
    except BaseException:
        for path in created:path.unlink(missing_ok=True)
        raise
    print(f'BIN: {out}\nLOG: {out.with_suffix(".log")}\nJSON: {out.with_suffix(".json")}')
    print(report['status'])
    return 0


if __name__=='__main__':
    try:
        raise SystemExit(main())
    except (PatchError,OSError,ValueError) as exc:
        print(f'[ERROR] {exc}',file=sys.stderr)
        raise SystemExit(2)
    except KeyboardInterrupt:
        print('\nCancelled',file=sys.stderr)
        raise SystemExit(130)
