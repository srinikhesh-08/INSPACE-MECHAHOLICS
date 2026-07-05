## Communication and Data Handling Subsystem
### CDH SYSTEM ARCHITECTURE

### Architecture Overview
The STM32F405 is the sole flight computer. All sensors, peripherals, and storage are subordinate devices. It manages sensor acquisition, calibration, FSM execution, packet formation, LoRa control, and microSD logging.

### Dual-Path Data Flow

### Primary Telemetry Path:
```
Sensors → STM32F405 → Packet Formation
→ SX1276 LoRa → Ground Control Station 
```

### Parallel Local Storage Path:
```
Sensors → STM32F405 → FAT32 microSD
```
Both paths operate concurrently. Data is preserved even if the LoRa link fails.

### M5Stack Camera (OV2640)
Connects via UART. Stores images locally only — does not transmit over LoRa. Operates at 2.4 GHz, providing RF isolation from the 868 MHz link.

### Peripheral Interface Assignments
<img width="259" height="131" alt="image" src="https://github.com/user-attachments/assets/3000255c-5db0-4634-9e3c-2bb210d510cf" />

### Communication Protocol- SX1276 LoRa

### LoRa Configuration Parameters
| **LoRa Configuration Parameters** |
|-----------------|
| Operating Band: 865–867 MHz (868 MHz nominal) |
| Bandwidth: 125 kHz |
| Spreading Factor: SF7 |
| Coding Rate: 4/5 |
| TX Power: 17 dBm |
| Preamble: 8 Symbols, Sync Word: 0x12 (Private Network) |
| CRC: Enabled, Mode: Half-duplex, Rate: 1 Hz |

### Design Rationale

SF7 at 125 kHz prioritises throughput and reduced air time. Link budget analysis confirms 56 dB margin at 1 km LOS — substantially exceeding operational requirements.

The private sync word 0x12 isolates the CanSat link from other LoRa devices at shared launch events.

Hardware CRC provides physical-layer error detection. This is the first of four independent integrity layers in the system.

Half-duplex uses dedicated TX/RX windows. The CanSat enters LAUNCH_PAD on power-up and awaits CMD_START_TELEMETRY before activating the downlink, preventing premature RF emissions.

### Telemetry Packet Content (~73 bytes, fixed layout)
- Mission Time, Packet Count, Flight State
- Altitude, Pressure, Temperature
- Battery Voltage, System Health Flags
- GPS: latitude, longitude, altitude, satellite count
- IMU: acceleration X/Y/Z, angular velocity X/Y/Z, orientation
Fixed-length binary packed C structure — deterministic GCS parsing.

### Communication Link Budget Analysis

```
**Step 1 — Free-Space Path Loss**
Signal attenuation due to geometric spreading over the link distance (unobstructed LOS):
FSPL(dB) = 20·log₁₀(d) + 20·log₁₀(f) + 32.44
d = distance in km, f = frequency in MHz
FSPL = 20·log(1) + 20·log(867) + 32.44
= 0 + 58.76 + 32.44 ≈ 91 dB

**Step 2 — Received Power**
Pr = Pt + Gt + Gr − FSPL
Pt = 17 dBm (TX power)
Gt = 2 dBi (CanSat monopole)
Gr = 5 dBi (GCS collinear)
Pr = 17 + 2 + 5 − 91 = −67 dBm

**Step 3 — Link Margin**
LM = Pr − Sensitivity
= −67 − (−123) = 56 dB
```

#### What Does 56 dB Mean?
The received signal is 56 dB above the SX1276 demodulation floor. Every 6 dB doubles tolerable degradation:

- Antenna orientation loss (monopole perpendicular to link): 10–15 dB — fully covered
- Typical multipath fading: 10–20 dB — fully covered
- Launch-site interference from adjacent LoRa teams: covered
- Conservative values used (2 dBi / 5 dBi) — actual margin likely higher

SF7 confirmed appropriate for 1 km LOS profile. Adjustable to higher SF if ground testing reveals unexpected degradation.

| **Summary** |
|-----------------|
| TX: 17 dBm |
| CanSat: 2 dBi |
| GCS: 5 dBi |
| FSPL @ 1 km |
| 867 MHz ≈ 91 dB |
| Pr = −67 dBm |
| Sensitivity: −123 dBm, Link Margin: 56 dB — Robust |

---

### Antenna System Design

#### CanSat — Quarter-Wave Monopole
Omnidirectional azimuthal radiation pattern selected because CanSat orientation cannot be controlled during descent.

##### Length Calculation:
L = c / (4 × f)
c = 3 × 10⁸ m/s, f = 867 MHz
L = (3×10⁸) / (4×867×10⁶) ≈ 8.6 cm
Copper conductor selected for high conductivity, low mass, and ease of fabrication. PCB ground plane serves as counterpoise.

#### CanSat Antenna Specifications
Type: Quarter-wave whip monopole
Material: Copper wire conductor
Frequency: 865–867 MHz
Gain: 2–3 dBi
Polarisation: Vertical
Pattern: Omnidirectional (azimuthal)
Length: ≈ 8.6 cm
Mounting: Above PCB ground plane (counterpoise)

#### GCS Antenna
Fiberglass-enclosed collinear, selected for increased gain while maintaining omnidirectional coverage without active tracking. Compensates for the CanSat's constrained installation and contributes directly to the 56 dB link margin.

GCS Antenna Specifications
1. Type: Fiberglass omnidirectional collinear
2. Material: Radiating elements enclosed in fiberglass radome
3. Frequency: 865–867 MHz
4. Gain: 5–8 dBi
5. Polarisation: Vertical
6. Pattern: Omnidirectional
7. Mounting: Fixed tripod or mast
8. Tracking: Not required

#### Polarisation Matching
Both antennas use vertical polarisation. Matched polarisation minimises polarisation loss — a mismatch would contribute 3–6 dB of avoidable attenuation.

##### Why No Tracking?
The 56 dB margin provides sufficient tolerance for signal variation caused by CanSat rotation over the 1 km mission profile. Active tracking hardware adds mass and complexity without a measurable benefit given the available margin.

---

### Telecommand Architecture — Uplink Design
| 6-Byte Command Packet |
| Byte 0: Sync Byte — 0xAA |
| Bytes 1–2: Team Number (uint16_t) |
| Byte 3: Opcode — command identifier |
| Byte 4: Parameter — command argument |
| Byte 5: XOR Checksum over Bytes 0–4 |

#### ACK / NACK Protocol
- Valid command (all layers passed + FSM state compatible) → ACK
- Invalid/incompatible → NACK
- Distinguishes a rejected command from a lost packet, providing diagnostic information that a timeout-only approach cannot supply

#### Supported Commands
| Opcode | Command |
|----------|---------|
| 0x01 | CMD_START_TELEMETRY |
| 0x02 | CMD_STOP_TELEMETRY |
| 0x03 | CMD_CALIBRATE_BARO |
| 0x04 | CMD_CALIBRATE_GYRO |
| 0x05 | CMD_CALIBRATE_ACCEL |
| 0x06 | CMD_ENTER_TEST |
| 0x07 | CMD_ENTER_LAUNCHPAD |
| 0x08 | CMD_SWITCH_CHANNEL |

#### 3-Layer Validation

#### Layer 1 — Sync Byte (0xAA)

- Rapid pre-filter
- Discards malformed packets, truncated transmissions, and frames from unrelated LoRa devices
- First position minimizes rejection cost

#### Layer 2 — Team Number

- Multiple teams may operate LoRa ground stations on the same frequency
- The uint16_t team identifier acts as a logical address layer
- Prevents cross-team command interference

#### Layer 3 — XOR Checksum

- XOR computed over Bytes 0–4
- Receiver independently recomputes and compares checksum
- Any bit corruption causes mismatch and packet rejection
- Provides a second integrity check above LoRa hardware CRC

The three layers defend against: noise-induced bit errors, cross-team interference, and corrupted packet execution. No single layer is sufficient in isolation.

---

### Data Acquisition System

#### BMP585 — Barometric Pressure Sensor

Provides atmospheric pressure and temperature measurements. Altitude is derived using the barometric formula. Serves as the primary sensor for flight state detection and apogee identification.

| Parameter | Value |
|------------|--------|
| Interface | I²C on PB6/PB7 |
| Address | Unique device address |
| Power | 0.000016 W continuous |

#### BNO086 — Inertial Measurement Unit

Provides 3-axis acceleration, angular velocity, and fused orientation (roll, pitch, yaw). The onboard fusion processor reduces STM32F405 computational load.

| Parameter | Value |
|------------|--------|
| Interface | I²C on PB6/PB7 |
| Bus Sharing | Shares bus with BMP585 |
| Power | 0.059 W continuous |

#### NEO-M9N — GNSS Receiver

Provides GPS positioning (latitude, longitude, altitude) and UTC time. UTC serves as the mission elapsed time reference independent of the onboard crystal oscillator.

| Parameter | Value |
|------------|--------|
| Interface | UART1 on PA9 (TX) / PA10 (RX) |
| Power | 0.224 W continuous |

#### Adaptive Sampling Strategy

| Device | Flight Phase | Sampling Rate | Purpose |
|----------|-------------|---------------|----------|
| BMP585 | Launch Pad | 5 Hz | Baseline monitoring |
| BMP585 | Ascent | 20 Hz | Apogee detection |
| BNO086 | Initialization | 10 Hz | Calibration and baseline establishment |
| BNO086 | Ascent | 100 Hz | Capture deployment and attitude transients |
| NEO-M9N | All Phases | 1–5 Hz | Position tracking |
| Telemetry Downlink | All Phases | 1 Hz | LoRa transmission rate |

The 100 Hz IMU sampling rate during ascent is a key design decision, ensuring high-frequency dynamic events are captured without aliasing.

---

### Flight State Machine — 7-State Mission FSM

#### FSM State Transitions

| From State | To State | Transition Condition |
|------------|-----------|---------------------|
| BOOT | TEST_MODE | Initialization sequence completed |
| TEST_MODE | LAUNCH_PAD | All subsystem health checks passed |
| LAUNCH_PAD | ASCENT | Sustained acceleration above threshold and positive altitude rate |
| ASCENT | PRIMARY_DESCENT | Apogee detected and sustained negative vertical velocity |
| PRIMARY_DESCENT | SECONDARY_DESCENT | Altitude below 600 ± 10 m AGL |
| SECONDARY_DESCENT | IMPACT | Near-ground altitude and IMU deceleration spike |
| IMPACT | Terminal State | Recovery mode and data preservation active |

#### Configurable Threshold Philosophy

Only the 600 ± 10 m AGL threshold is finalized.

Reasons for deferring other thresholds:

- Thresholds depend on aerodynamic and structural characteristics of the final assembly
- Reliable prediction is not possible without empirical testing
- Premature commitment risks invalid assumptions due to hardware modifications or parachute behavior
- Configurable parameters allow updates without firmware recompilation

#### Qualification Process

1. 6-DOF simulation to establish initial estimates
2. Bench testing of individual sensor responses
3. Integrated ground testing of the complete FSM
4. Flight validation and refinement
5. Final values documented during CDR

#### Boot Sequence

```text
Power
→ Sensor Initialization
→ Bus Initialization (SPI1, SPI2, I²C, UART1)
→ SD Mount
→ Telemetry Initialization
→ Health Check
→ State Restore
→ BOOT
→ TEST_MODE
→ LAUNCH_PAD
→ Await CMD_START_TELEMETRY
```
---

### CDH Subsystem Power Budget
P = V × I
| Symbol | Description |
| ------ | ----------- |
| P      | Power (W)   |
| V      | Voltage (V) |
| I      | Current (A) |

Example: 
V = 3.3 V
I = 110 mA = 0.110 A
P = 3.3 × 0.110
P = 0.363 W
All subsystem power values are derived using the same method and datasheet specifications.

#### SX1276 Duty Cycle Analysis
- Half-duplex operation at 1 Hz
- TX occupies approximately 20% of each cycle
- Idle occupies approximately 80% of each cycle
- 
TX Average   = 0.396 W × 0.20 = 0.079 W
Idle Average = 0.033 W × 0.80 = 0.026 W

Weighted Average ≈ 0.105 W

##### M5Stack Camera

At 0.80 W, the M5Stack Camera is the largest single CDH power consumer. Its intermittent operation during descent limits total energy consumption over the mission duration.

#### CDH POWER CONSUMPTION TABLE
| Component      | Voltage | Current  | Power      | Duty Cycle   |
| -------------- | ------- | -------- | ---------- | ------------ |
| STM32F405      | 3.3 V   | 110 mA   | 0.363 W    | Continuous   |
| BMP585         | 3.3 V   | 0.005 mA | 0.000016 W | Continuous   |
| BNO086         | 3.3 V   | 17.9 mA  | 0.059 W    | Continuous   |
| NEO-M9N        | 3.3 V   | 67.9 mA  | 0.224 W    | Continuous   |
| SX1276 TX      | 3.3 V   | 120 mA   | 0.396 W    | 20% Duty     |
| SX1276 Idle    | 3.3 V   | 10 mA    | 0.033 W    | 80% Duty     |
| microSD        | 3.3 V   | 50 mA    | 0.165 W    | Continuous   |
| M5Stack Camera | 5.0 V   | 160 mA   | 0.800 W    | Intermittent |

---

### Data Processing, Integrity and Storage

##### Onboard Processing

STM32F405 performs:

1. Sensor acquisition
2. Unit conversion
3. Calibration
4. FSM execution
5. Telemetry packet assembly

- Active FSM state determines sampling rates and operational behavior.

##### Packet Formation

- Fixed-layout binary packed C structures
- Approximately 73 bytes per packet
- Deterministic GCS parsing
- No framing ambiguity

#### Data Integrity - 4 Layers

##### Layer 1 - LoRa Hardware CRC

- Physical-layer bit error detection performed by SX1276 hardware.

##### Layer 2 - Application XOR Checksum

- Computed before transmission.
- Verified upon reception.
- Invalid packets discarded before processing.

##### Layer 3 - Sequential Packet Counter

- Incremented for every packet.
- Enables gap detection and continuity monitoring.

##### Layer 4 - Telecommand Validation

- Sync Byte validation.
- Team ID validation.
- XOR checksum validation.
- ACK/NACK closes the command loop.

#### Storage Architecture

- FAT32 microSD connected via SPI1 (PA5-PA7, PA4 CS).

Stored data includes:

1. Telemetry data (sensor readings and computed values)
2. Mission state transitions (FSM state logs)
3. Recovery information (GPS landing coordinates)

#### Mission Continuity After Reset

The following values are continuously persisted to microSD:

1. Mission elapsed time
2. Packet counter
3. Current flight state

Upon reset, these values are restored before mission execution resumes.

Benefits:

1. Packet numbering continuity
2. Mission time consistency
3. State preservation

#### Mission Timing

- NEO-M9N GPS UTC time provides an absolute mission time reference independent of the onboard crystal oscillator.
- Eliminates long-term clock drift errors.
