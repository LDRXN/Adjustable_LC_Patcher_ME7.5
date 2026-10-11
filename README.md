# 🚀 Bosch ME7.5 Adjustable Launch Control Patcher

An open-source patcher for installing **Adjustable Launch Control** on compatible **Bosch ME7.5 1MB** ECU firmware.

Unlike a conventional Launch Control patch with a permanently fixed RPM limit, this implementation allows the driver to **change and store the Launch Control RPM directly from the vehicle controls**.

The patcher analyzes the supplied ECU firmware together with its matching **ME7Info `.ecu` definition**, automatically resolves the required RAM signals and firmware hooks, injects the Adjustable LC code into an available code area, and creates a new modified BIN without overwriting the original firmware.

---

# ✨ Features

* ✅ Adjustable Launch Control RPM
* ✅ Driver-programmable LC RPM
* ✅ Default LC RPM configuration
* ✅ Configurable minimum and maximum LC RPM
* ✅ Configurable vehicle-speed activation threshold
* ✅ Two-stage ignition retard strategy
* ✅ Configurable ignition retard angles
* ✅ Configurable RPM windows
* ✅ Ignition / output cut above the configured LC threshold
* ✅ Brake and clutch control sequence
* ✅ Tachometer / RPM needle confirmation during RPM programming
* ✅ Automatic ECU signal resolution from matching `.ecu`
* ✅ Automatic firmware hook detection
* ✅ Automatic code-cave detection
* ✅ Automatic Bosch HW/SW identity verification
* ✅ Automatic BIN / ECU definition matching validation
* ✅ Existing Adjustable LC patch detection
* ✅ Generates detailed `.log`
* ✅ Generates detailed `.json`
* ✅ Dry-run analysis mode
* ✅ Optional automatic checksum correction using `me7sum`
* ✅ Original BIN is never overwritten
* ✅ Open Source

---

# 🚗 Supported ECU Type

The patcher is designed for:

* Bosch **ME7.5**
* **1MB / 1048576-byte firmware**
* Four-cylinder applications using the supported ME7.5 firmware structures
* Firmware for which a matching **ME7Info `.ecu`** definition is available

The patcher does **not** use a fixed table of ECU-specific addresses.

Instead, important signals are resolved dynamically from the supplied `.ecu` file and the required firmware locations are detected by analyzing supported instruction structures inside the BIN.

This makes the patch significantly more portable between compatible ME7.5 software versions.

> Compatibility with every Bosch ME7.5 software version is not guaranteed.  
> Unsupported or unknown firmware structures are rejected instead of being patched blindly.

---

# 📁 Required Files

The patcher requires:

```text
ECU_Firmware.bin
ECU_Firmware.ecu
```

The `.ecu` file must correspond to the exact firmware being modified.

Example:

```text
06A906032DR.bin
06A906032DR.ecu
```

If the `.ecu` file has the same base filename as the BIN, the application can detect it automatically.

If there is only one `.ecu` file in the directory, it can also be selected automatically.

Otherwise, specify it manually using:

```text
--ecu
```

Example:

```bash
python Adjustable_LC_Patcher_v2.py test.bin --ecu test.ecu
```

---

# 🔍 BIN / ECU Verification

Before installing the patch, the application verifies that the supplied `.ecu` definition belongs to the loaded firmware.

The verification includes Bosch identification information such as:

```text
HW Number
SW Number
Part Number
SW Version
```

The patch is aborted if the identification contained in the `.ecu` file does not match the BIN.

This helps prevent using memory addresses and signal definitions from an incorrect ECU software version.

---

# 🧠 Automatic Signal Detection

The patcher reads the required runtime signals directly from the matching **ME7Info `.ecu` definition**.

Important signals include:

* 📍 Engine RPM
* 📍 Vehicle speed
* 📍 Clutch switch
* 📍 Brake condition
* 📍 Tachometer / RPM display control
* 📍 Knock-control condition
* 📍 Engine runtime timing signal
* 📍 Ignition angle
* 📍 Individual cylinder ignition outputs

For the four-cylinder ignition strategy, the patcher also validates the layout of:

```text
zwoutar_0
zwoutar_1
zwoutar_2
zwoutar_3
```

Firmware using an unsupported ignition-output layout is rejected.

---

# 🔧 Automatic Firmware Hook Detection

The patcher does not rely on hardcoded flash addresses taken from one specific ECU file.

Instead, it searches the BIN for complete supported Bosch instruction structures associated with the required functions.

The patcher resolves hooks for:

* ⚙️ Periodic LC logic
* 🔥 Ignition-angle control
* ✂️ Output cut control
* 📈 Tachometer / RPM indication
* 🧠 Knock-control branch
* 📈 Tachometer / RPM output

Each required structure must be detected uniquely.

If the expected structure cannot be found, or if multiple ambiguous matches are found, the patcher stops without modifying the firmware.

---

# 🎯 Adjustable Launch Control

The main feature is the ability to change the Launch Control RPM without creating a new BIN every time.

The driver can enter the LC programming sequence using the brake and clutch pedals and then capture the desired engine RPM.

The stored target RPM is subsequently used by the Launch Control logic.

---

# 🎮 RPM Programming Sequence

The Adjustable LC programming sequence uses:

```text
2 × Brake releases
3 × Clutch releases
```

After completing the sequence, the ECU enters RPM capture mode.

When programming mode is entered successfully, the **tachometer needle moves to approximately 7000 RPM for about 1 second**, then returns to displaying the **actual engine RPM**.

The driver can then bring the engine to the desired RPM while the tachometer continues to display the real engine speed.

Pressing the **brake** confirms and stores the current RPM as the new Launch Control target.

After the new LC RPM is successfully stored, the **tachometer needle again moves to approximately 7000 RPM for about 1 second**, then returns to the actual engine RPM.

Conceptually:

```text
2 × Brake
      ↓
3 × Clutch
      ↓
RPM Capture Mode
      ↓
Tachometer → ~7000 RPM for ~1 second
      ↓
Return to actual engine RPM
      ↓
Select desired engine RPM
      ↓
Press Brake
      ↓
New Launch Control RPM stored
      ↓
Tachometer → ~7000 RPM for ~1 second
      ↓
Return to actual engine RPM
```

The RPM must remain inside the configured allowed range.

If the captured RPM is outside the allowed range, the configured default RPM is used instead.

---

# 📈 Tachometer / RPM Indication

The Adjustable LC programming mode uses the vehicle's **tachometer needle** as visual confirmation.

When the programming sequence is completed successfully:

```text
Tachometer → approximately 7000 RPM
Duration   → approximately 1 second
Then       → returns to actual engine RPM
```

The engine continues to operate normally and the tachometer displays the real engine speed while the driver selects the desired Launch Control RPM.

When the driver presses the **brake** to confirm and store the selected RPM, the same confirmation movement is performed again:

```text
New LC RPM stored
      ↓
Tachometer → approximately 7000 RPM for ~1 second
      ↓
Return to actual engine RPM
```

This provides a clear confirmation for both:

* entering Adjustable LC programming mode;
* successfully saving the new Launch Control RPM.

---

# 🚦 Launch Control Activation Conditions

Launch Control operates only when the required conditions are satisfied.

The primary conditions include:

* ✅ Clutch condition active
* ✅ Vehicle speed below the configured threshold
* ✅ Valid stored LC RPM
* ✅ Engine operating above the minimum running threshold

If the vehicle exceeds the configured LC speed threshold, the Adjustable LC sequence/state is reset.

This prevents the low-speed Launch Control strategy from remaining active during normal driving.

---

# 🔥 Ignition Retard Strategy

The patch implements two configurable ignition-retard zones around the selected Launch Control RPM.

Default configuration:

```text
Lower RPM window : 100 RPM
Upper RPM window : 100 RPM

Lower angle      : -20.25°
Upper angle      : -30.00°
```

This creates progressively more aggressive ignition retard as engine speed approaches and passes the selected LC target.

Conceptually:

```text
Target RPM - Lower Window
        │
        ├── Lower ignition retard
        │
Target RPM
        │
        ├── Upper ignition retard
        │
Target RPM + Upper Window
        │
        └── Output cut
```

The ignition retard is applied to the individual four-cylinder ignition outputs as well as the corresponding ignition-angle output used by the supported ECU structure.

---

# ✂️ Output Cut

When engine speed reaches the configured upper LC threshold, the patch activates the Launch Control cut strategy.

The cut threshold is based on:

```text
Selected LC RPM + Upper RPM Window
```

For example:

```text
Selected LC RPM : 3500 RPM
Upper Window    : 100 RPM

Cut threshold   : approximately 3600 RPM
```

If the selected LC RPM is changed to:

```text
4000 RPM
```

the corresponding threshold becomes approximately:

```text
4100 RPM
```

Therefore, the cut point moves dynamically with the driver's selected Launch Control RPM.

---

# ⚙️ Default Configuration

The current default configuration is:

| Parameter | Default |
|---|---:|
| 🎯 Default LC RPM | **3500 RPM** |
| 🔽 Minimum selectable RPM | **1500 RPM** |
| 🔼 Maximum selectable RPM | **5500 RPM** |
| 🚗 LC speed threshold | **3.75 km/h** |
| 🔥 Lower RPM window | **100 RPM** |
| 🔥 Upper RPM window | **100 RPM** |
| ⚡ Lower ignition angle | **-20.25°** |
| ⚡ Upper ignition angle | **-30°** |
| ⏱️ Programming sequence timeout | **80 × 100 ms** |
| 📈 Tachometer confirmation duration | **~1 second** |
| ⚙️ Engine running threshold | **500 RPM** |

The default programming-sequence window is therefore approximately:

```text
8 seconds
```

and the tachometer confirmation movement lasts approximately:

```text
1 second
```

---

# 🔧 Configurable Parameters

The patcher allows the following parameters to be changed:

```text
--rpm
--min-rpm
--max-rpm
--speed
--lower-window
--upper-window
--lower-angle
--upper-angle
--sequence-ticks
--engine-min-rpm
```

Example:

```bash
python Adjustable_LC_Patcher_v2.py test.bin \
  --ecu test.ecu \
  --defaults \
  --rpm 4000 \
  --speed 5
```

This would configure:

```text
Default LC RPM     : 4000 RPM
LC speed threshold : 5 km/h
```

while retaining the remaining default parameters.

---

# 🚀 Interactive Usage

Run:

```bash
python Adjustable_LC_Patcher_v2.py
```

The program asks for the BIN:

```text
BIN file:
```

After loading the BIN and matching `.ecu`, the interactive configuration provides:

```text
[1] Reference defaults: 3500 RPM, 1500–5500 RPM range, 3.75 km/h
[2] Change default RPM and LC speed threshold
```

Selecting:

```text
1
```

uses the default configuration.

Selecting:

```text
2
```

allows the user to enter:

```text
Default LC RPM
LC speed threshold
```

The remaining parameters retain their predefined values unless they are explicitly supplied from the command line.

---

# 💻 Command-Line Usage

## Default Configuration

```bash
python Adjustable_LC_Patcher_v2.py test.bin \
  --ecu test.ecu \
  --defaults
```

---

## Custom Default RPM

```bash
python Adjustable_LC_Patcher_v2.py test.bin \
  --ecu test.ecu \
  --defaults \
  --rpm 4000
```

---

## Custom RPM and Speed

```bash
python Adjustable_LC_Patcher_v2.py test.bin \
  --ecu test.ecu \
  --defaults \
  --rpm 4000 \
  --speed 5
```

---

## Complete Example

```bash
python Adjustable_LC_Patcher_v2.py test.bin \
  --ecu test.ecu \
  --defaults \
  --rpm 4000 \
  --min-rpm 2000 \
  --max-rpm 5500 \
  --speed 5 \
  --lower-window 100 \
  --upper-window 100 \
  --lower-angle -20.25 \
  --upper-angle -30
```

---

# 🧪 Dry Run

The patcher contains a dry-run mode that performs the firmware analysis and generates the complete patch in memory without writing any files.

Example:

```bash
python Adjustable_LC_Patcher_v2.py test.bin \
  --ecu test.ecu \
  --defaults \
  --dry-run
```

The application analyzes:

* ECU identification
* Required `.ecu` signals
* Firmware hooks
* RAM candidate
* Code cave
* LC parameters
* Generated patch structure

but does not create a modified BIN.

At the end:

```text
ANALYSIS ONLY — no files written.
```

This mode is recommended before creating the final patched firmware.

---

# 🧠 RAM Allocation

Adjustable LC requires private runtime state in ECU RAM.

The patch reserves:

```text
32 bytes
```

for its runtime state.

The preferred method is to provide an explicitly reserved measurement in the `.ecu` definition:

```text
LC_RAM
```

with at least:

```text
Size: 32 bytes
Mask: 0
```

Alternatively, RAM can be specified manually using:

```text
--ram-base
```

The patcher can also search for a candidate unused symbol gap, but this should be treated as a **candidate rather than proof that the RAM area is completely unused**.

Indirect or dynamically calculated RAM accesses cannot be completely proven safe by static analysis.

---

# 🕳️ Automatic Code Cave Detection

The injected C166 code is placed inside a suitable unused flash area containing erased bytes.

The patcher automatically searches for an aligned code cave large enough to contain:

* Adjustable LC code
* Runtime wrappers
* Patch metadata
* Identification marker

The patch also verifies that the generated code remains inside the required C166 code segment boundaries.

---

# 🛡️ Existing Patch Protection

The application refuses to install Adjustable LC if it detects:

* its own existing patch marker;
* a supported existing reference Adjustable LC implementation.

This prevents accidentally stacking multiple Launch Control patches on top of each other.

Always use an **unpatched original or compatible base BIN** when installing Adjustable LC.

---

# 📁 Output Files

By default, the patcher creates:

```text
test_ADJUSTABLE_LC_BENCH.bin
test_ADJUSTABLE_LC_BENCH.log
test_ADJUSTABLE_LC_BENCH.json
```

If these files already exist, the patcher automatically uses another filename:

```text
test_ADJUSTABLE_LC_BENCH_2.bin
test_ADJUSTABLE_LC_BENCH_2.log
test_ADJUSTABLE_LC_BENCH_2.json
```

The original BIN is **never overwritten**.

The output is created exclusively as a new file to reduce the risk of accidentally replacing the source firmware.

---

# 📄 Detailed LOG

The generated `.log` contains technical information including:

* ECU identity
* Resolved signals
* Signal addresses
* Signal conversion factors
* Detected firmware hooks
* Original hook bytes
* Injected wrapper addresses
* Brake detection information
* Code-cave range
* RAM allocation
* LC configuration
* Encoded raw parameters
* Timing information
* Integration details
* Validation limitations
* Input SHA256
* Output SHA256

This makes it possible to inspect exactly how the patch was generated for the selected ECU firmware.

---

# 📋 JSON Report

In addition to the human-readable log, the application creates a structured:

```text
.json
```

report.

The JSON report contains detailed patch metadata that can be used for:

* development;
* automated verification;
* comparison between firmware variants;
* debugging;
* integration with other ME7 tools.

---

# ✅ Parameter Validation

The patcher validates the configured parameters before generating the firmware.

Among other checks:

```text
1000 <= Minimum RPM <= Default RPM <= Maximum RPM <= 8000
```

Vehicle speed must be:

```text
> 0 and <= 20 km/h
```

Ignition retard angles must remain within:

```text
-90° ... 0°
```

The application also validates RPM windows, timer values and conversion limits before encoding them into ECU-native values.

Invalid configurations are rejected.

---

# ⚠️ Checksum

The patcher can optionally use **me7sum** to correct the checksum.

Specify it manually:

```bash
python Adjustable_LC_Patcher_v2.py test.bin \
  --ecu test.ecu \
  --defaults \
  --me7sum /path/to/me7sum
```

The program also searches for:

```text
me7sum
me7sum.exe
```

in the patcher's directory and the input BIN directory.

On Linux, a Windows:

```text
me7sum.exe
```

can be executed automatically through **Wine** if available.

Checksum correction is performed **after** the Adjustable LC patch has been generated.

The application then verifies that the checksum utility did not modify the injected LC code or firmware hook bytes.

If `me7sum` is not available, the generated firmware remains checksum-uncorrected.

> **Always verify and correct the ECU checksum before flashing.**

---

# 🔒 OEM Integration

The Adjustable LC code is integrated with the existing Bosch output-control strategy instead of simply replacing the OEM logic.

Existing OEM output-disable requests retain priority.

The patch overlays the required Launch Control cut condition while preserving the existing factory control path when LC conditions are not satisfied.

The implementation also contains handling for the reference `B_kr` branch and the tachometer / RPM confirmation output path.

---

# 🚫 Not Included

This is a standalone **Adjustable Launch Control** implementation.

It does **not** install:

* ❌ MapSwitch
* ❌ NLS / No-Lift Shift
* ❌ Pops & Bangs
* ❌ Rolling Anti-Lag

Only the Adjustable LC functionality is installed.

---

# ⚠️ BENCH Status

The current patcher is explicitly marked as:

```text
BENCH
```

This is important.

The software performs extensive static validation of:

* BIN identity;
* `.ecu` identity;
* firmware structures;
* hook locations;
* signal layout;
* RAM candidates;
* code cave;
* generated C166 code;
* modified byte regions.

However, static firmware analysis cannot prove complete hardware or vehicle compatibility.

In particular:

* compatibility with every ME7.5 software version is **not guaranteed**;
* automatically selected RAM is a candidate and does not prove absence of every indirect runtime access;
* cluster / CAN behavior of the tachometer / RPM indication requires ECU / vehicle verification;
* hardware timing and real vehicle behavior cannot be validated by the patcher itself.

Unsupported firmware should be rejected rather than patched using guessed addresses.

---

# 🛡️ Disclaimer

This software is intended for **educational, research, development and bench-testing purposes**.

The user is solely responsible for:

* selecting the correct ECU firmware;
* supplying the correct matching `.ecu` file;
* reviewing detected signals and firmware hooks;
* ensuring that the selected RAM area is safe;
* validating the generated firmware;
* correcting and verifying checksums;
* testing the modification safely before vehicle use.

Incorrect ECU firmware modifications can result in improper engine operation or ECU failure.

💾 Always keep an untouched backup of the original firmware.

---

# 🤝 Contributing

Contributions are welcome!

Feel free to:

* 🐞 Report bugs
* 💡 Suggest improvements
* 🔧 Add support for additional ME7.5 software variants
* 🔍 Improve firmware detection
* 🧠 Improve RAM validation
* 🚗 Provide verified test results
* 🔧 Submit pull requests

---

# 📜 License

Open Source.

See the repository license for details.

---

## Software License

Copyright © 2026 LDRXN

This software is free and open-source software distributed under the terms of the **GNU General Public License, version 3 (GNU GPL v3)**.

Every user is permitted to:

- use the software free of charge for private and commercial purposes;
- copy and redistribute the original software;
- access the source code;
- study and modify the source code;
- create modified versions of the software;
- distribute modified versions of the software.

When publicly distributing a modified version of the software, the applicable copyright and license notices must be preserved and the corresponding source code must be made available under the terms of the GNU GPL v3.

The software is provided **“AS IS”**, without warranty, to the extent permitted by applicable law. LDRXN provides no warranty regarding the software's fitness for a particular purpose or error-free operation.

This notice is a summary of the licensing terms. The complete GNU General Public License version 3 supplied with the software constitutes the applicable license terms.

---

<h2 align="center">❤️ Want to say thank you?</h2>

<p align="center">
If this project helped you and you would like to show your appreciation,<br>
<strong>there is no need to support me financially.</strong>
</p>

<p align="center">
Instead, if you are willing and able, please consider supporting organizations<br>
that help children and people fighting serious illnesses.
</p>

<table align="center">
<tr>
<td align="center">

<a href="https://www.siepomaga.pl/">
<img src="https://img.shields.io/badge/SUPPORT-SIEPOMAGA.PL-ff4f81?style=for-the-badge" alt="Support Siepomaga.pl">
</a>

</td>
<td align="center">

<a href="https://cancerfighters.pl/">
<img src="https://img.shields.io/badge/SUPPORT-CANCER%20FIGHTERS-e30613?style=for-the-badge" alt="Support Cancer Fighters">
</a>

</td>
</tr>
</table>

<p align="center">
<strong>Thank you for using my project. ❤️</strong><br><br>
If it helped you, consider helping someone who truly needs it.<br>
Even a small contribution can make a meaningful difference.
</p>

---
