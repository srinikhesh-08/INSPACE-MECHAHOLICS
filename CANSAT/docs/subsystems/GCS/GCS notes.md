# CanSat - CDH

## Contents

- GCS Hardware and Software Architecture
- GCS Mission Status Panel and Command Interface
- GCS Real-Time Displays and Attitude Visualisation
- GCS Logging, Recovery Interface and Field Deployment
- Telemetry and Telecommand Packet Structure
- Communication Validation Architecture — 4-Layer Defence
- Boot Sequence — 9-Step Initialisation Workflow (Finalised)
- Mission Continuity and Reset Recovery
- System Design Justification — Consolidated Engineering Rationale
- Preliminary Link Budget Analysis
- Mission Timing Architecture — GNSS-Synchronised
- Transmission Loss Mitigation and Backup RF Capability




---
# GCS Hardware and Software Architecture


- GCS Hardware Configuration

    - Laptop Computer:                      Primary processing, display, and logging platform
    - SX1276-compatible LoRa receiver:      RF reception and telecommand uplink transmission
    - USB-to-UART interface:                Bridge between LoRa receiver and laptop software
    - Fiberglass collinear antenna:         865–867 MHz, 5–8 dBi gain, vertical polarisation, tripod-mounted
    - Portable field power source:          Independent operation — no mains electricity required

    - Configured: 865–867 MHz | SF7 | BW 125 kHz | CR 4/5 | Sync 0x12


- GCS Software Stack

    - Python — Core application language:           Cross-platform; serial I/O, data processing, GUI integration
    - PyQt6 — GUI framework:                        Event-driven; multi-panel layouts, custom widgets
    - PyQtGraph — Real-time plotting:               OpenGL-accelerated; updates at 1 Hz telemetry rate
    - Pandas — Data processing:                     CSV logging; time-series operations; structured data
    - NumPy — Numerical computation:                Vertical velocity derivation; vectorised operations



---
# GCS Mission Status Panel and Command Interface


- Mission Status Panel

    - Persistent panel updating at 1 Hz with each incoming packet:

    - Mission Elapsed Time — from GPS UTC in telemetry
    - Packet Count — total received; gaps from sequence counter Mission State — reflects current CanSat FSM state
    - RSSI (dBm) — signal strength from LoRa receiver SNR (dB) — signal-to-noise ratio
    - Battery Voltage — real-time power health
    - Reset Count — processor resets since mission start Communication Link Status — active / lost / degraded


- Command Interface

    - XOR checksum auto-computed by GCS software — every packet structurally valid. ACK/NACK displayed per command.

    - 0x01 CMD_START_TELEMETRY — Activate downlink
    - 0x02 CMD_STOP_TELEMETRY — Suspend downlink
    - 0x03 CMD_CALIBRATE_BARO — Barometric zero
    - 0x04 CMD_CALIBRATE_GYRO — Gyro calibration
    - 0x05 CMD_CALIBRATE_ACCEL — Accel calibration
    - 0x06 CMD_ENTER_TEST — Transition to TEST_MODE
    - 0x07 CMD_ENTER_LAUNCHPAD — To LAUNCH_PAD
    - 0x08 CMD_SWITCH_CHANNEL — Switch radio channel


---
# GCS Real-Time Displays and Attitude Visualisation


- Real-Time Telemetry Display
    - Structured display by category, updating at 1 Hz:
        - Environmental
            - Altitude (m AGL), Pressure (hPa), Temperature (°C)
        - IMU
            - Acceleration X, Y, Z (m/s²) Angular velocity X, Y, Z (deg/s)
        - GPS
            - Latitude, Longitude (decimal degrees) GPS Altitude (m), Satellite count
        - Power and Communication
            - Battery voltage (V)
            - RSSI (dBm), SNR (dB), Packet count, Link status


- Attitude Visualisation Panel
    - Dedicated panel: Artificial horizon, roll, pitch, heading — driven by BNO086 orientation data. Continuous visual monitoring without manual        interpretation of raw angular rates.


- Real-Time Plots (PyQtGraph, 1 Hz)
    - Flight Data
        - Altitude vs. Time 
        - Pressure vs. Time
        - Temperature vs. Time
        - Vertical Velocity vs. Time (derived)
    - IMU
        - Acceleration X, Y, Z vs. Time 
        - Angular Velocity X, Y, Z vs. Time
    - Orientation
        - Roll, Pitch, Yaw vs. Time
    - GPS and Power
        - Latitude, Longitude, GPS Altitude, Satellite Count 
        - Battery Voltage vs. Time


---
# GCS Logging, Recovery Interface and Field Deployment

- Data Logging
    - Timestamped CSV files on local storage. Logged per packet:
        - Mission Time (ms since launch epoch) 
        - Packet Number
        - All sensor fields: altitude, pressure, temp, accel X/Y/Z, ang-vel X/Y/Z, roll, pitch, yaw
        - GPS: lat, lon, altitude, satellite count 
        - Mission State (FSM identifier)
        - RSSI (dBm), SNR (dB)

- GCS Mission Continuity
    - On GCS software restart, the application recovers mission time, packet count, and flight state from the last CSV log before resuming — maintaining GCS coherence after a laptop interruption.

- Recovery Interface
    - After CanSat transitions to IMPACT state:
        - Launch location — fixed reference 
        - Current GPS position — last received 
        - Landing position — last valid fix
        - Recovery guidance — direction and distance

- Field Deployment
    - Designed for rapid deployment by a small team: 
        - Position laptop at operations area 
        - Connect LoRa receiver via USB-to-UART 
        - Erect tripod, mount collinear antenna 
        - Connect portable power source
        - Launch GCS software; verify receiver configuration 
        - Issue CMD_START_TELEMETRY once CanSat powered

- Design Justification
    - Python/PyQt6/PyQtGraph: cross-platform, mature libraries, single application integrating serial handling, processing, and visualisation.
    - Automatic XOR checksum eliminates operator errors in packet construction. ACK/NACK provides unambiguous command confirmation.
    - 5–8 dBi collinear antenna is the primary ground-side contributor to the 56 dB link margin from the CDH link budget.


---
# Telemetry and Telecommand Packet Structure

- Telemetry Packet — 75 Bytes (Finalised)
    - 0 team_number uint16_t 2B
    - 2 mission_time uint32_t 4B 
    - 6 packet_count uint16_t 2B 
    - 8 altitude float 4B
    - 12 pressure float 4B 
    - 16 temperature float 4B 
    - 20 voltage	float 4B
    - 24 gnss_time  uint32_t 4B 
    - 28 latitude	float 4B
    - 32 longitude  float 4B 
    - 36 gnss_altitude float 4B
    - 40 gnss_sats  uint8_t 1B
    - 41 accel_x/y/z float x3 12B 
    - 53 roll, pitch float x2 8B
    - 61 gyro_x/y/z  float x3 12B
    - 73 flight_state uint8_t 1B
    - 74 checksum	uint8_t 1B
    
    - Earlier revisions referenced a 73-byte packet; the structure above (75 bytes) is the finalised PDR definition and supersedes that figure throughout this report.


- Telecommand Packet — 6 Bytes
    - Byte 0: Sync Byte = 0xAA
    - Bytes 1–2: Team Number (uint16_t)
    - Byte 3: Opcode
    - Byte 4: Parameter
    - Byte 5: XOR Checksum (Bytes 0–4)


- Key Design Points

    - Packed struct — no compiler padding; identical layout in firmware and GCS parser
    - Little-endian byte order (STM32F405 native) 
    - packet_count (offset 6) enables GCS gap detection
    - checksum (offset 74) is the application-layer XOR — Layer 4 of validation architecture
    - flight_state (offset 73) directly drives GCS Mission State display
    - 6-byte command format minimises uplink air time while retaining sync, source ID, and integrity check



---
# Communication Validation Architecture — 4-Layer Defence

- 4-Layer Validation Pipeline
    - Layer 1 — LoRa PHY Synchronisation
        - Applied to: all received frames. Addresses: bit errors from channel noise; frame misalignment
    - Layer 2 — Team Number Filtering
        - Applied to: all received packets. Addresses: cross-team command interference at shared launch sites
    - Layer 3 — CRC Validation
        - Applied to: all received packets. Addresses: residual bit errors not caught by PHY sync
    - Layer 4 — XOR Checksum Verification
        - Applied to: telemetry + telecommand. Addresses: application-layer corruption; end-to-end integrity
    - Telecommand-Only: Sync Byte (0xAA) + FSM-State Check
        - Final gates before command execution

- Why Layered Validation?
    - Each layer addresses a distinct failure mode. No single layer is sufficient in isolation — removing any one leaves the system vulnerable to the failure mode it specifically addresses.

- Telecommand-Specific Gates
    - Telecommand packets pass through two additional checks beyond the four layers:
        - Sync byte validation — Byte 0 must equal 0xAA
        - FSM-state compatibility — command must be valid in current state
    - Only packets satisfying all applicable conditions are accepted for processing. A command failing any check returns NACK; a valid command compatible with the current FSM state returns ACK.

- Outcome
    - This architecture defends simultaneously against: random channel noise, RF interference from co-located teams, packet collisions, and accidental or malicious cross-team commands — each addressed by a different layer.




---
# Boot Sequence — 9-Step Initialisation Workflow (Finalised)


- Boot Sequence — Finalised
    - The flight software boot sequence is finalised. Each step completes successfully before the next begins; a failure is recorded in the health diagnostic log and reported to the GCS once telemetry is established.

    - 1. Power Application
        - Main power rail established; regulators stabilise supply
    - 2. STM32F405 Initialisation
        - Flight computer core, clocks, internal peripherals operational
    - 3. Sensor Initialisation
        - BMP585, BNO086, NEO-M9N out of reset; registers configured
    - 4. Communication Bus Init
        - SPI1, SPI2, I²C, UART1 initialised at configured rates
    - 5. microSD Mounting
        - FAT32 mounted on SPI1; log files opened; write access verified
    - 6. Telemetry Subsystem Init
        - SX1276 configured; TX/RX buffers initialised
    - 7. Health Diagnostics
        - All subsystem self-tests executed; faults flagged
    - 8. Previous Mission-State Recovery
        - Persisted time, packet count, FSM state read from microSD
    - 9. Transition to BOOT Completion
        - BOOT → TEST_MODE → LAUNCH_PAD; awaits CMD_START_TELEMETRY

- Boot-Time Measurement — PDR Status
    - A measured boot duration has not yet been established, as integrated hardware testing has not been completed. The sequence itself (left) is fully finalised — only the timing measurement is pending.
    - Boot-time measurement will be finalised during integrated system testing and flight qualification activities, with the measured value reported at the Critical Design Review.

- Step 8 — Previous Mission-State Recovery
    - Entry point to the reset recovery workflow detailed on the next slide. Ensures the system resumes with the correct mission time, packet counter, and FSM state after any processor reset, during initial boot or mid-mission.

- Step 9 — Transition to BOOT Completion
    - The FSM completes BOOT, transitions through TEST_MODE on successful validation, then to LAUNCH_PAD. The system remains in LAUNCH_PAD — sensors active, telemetry link established, downlink not yet transmitting — until CMD_START_TELEMETRY is received from the GCS.


---
# Mission Continuity and Reset Recovery


- Persisted Mission Variables
    - Three variables are continuously written to microSD during operation, enabling coherent recovery after an unplanned processor reset:
        - Mission Elapsed Time
            - Written each telemetry cycle — provides time continuity after reset
        - Packet Counter
            - Written each telemetry cycle — GCS never sees a counter reset to zero
        - Current FSM State
            - Written on every state transition — correct operational state resumed

- Why This Matters
    - Without persisted state, a processor reset mid-mission would restart the packet counter from zero, reset mission time to zero, and force the FSM back to BOOT — producing a telemetry record the GCS cannot reconcile with pre-reset data.
    - Persisted state ensures the post-reset stream is a continuation, not a restart.
    - The GCS distinguishes a reset event from packet loss via the Reset Count field: a reset produces a small time gap with continuity preserved, whereas packet loss produces sequence-counter gaps with no time discontinuity.

- Reset Recovery Workflow
    - 1. Read Stored State
        - Mission state log file read from microSD
    - 2. Restore Packet Counter
        - Resume from last persisted packet_count value
    - 3. Restore Mission Time
        - Resume from last persisted mission_time value
    - 4. Restore FSM State
        - Resume in the last persisted flight_state
    - 5. Resume Nominal Operation
        - Continue sensor sampling, telemetry, and logging from restored context

    - This workflow executes as Step 7 of the boot sequence and applies identically whether the reset occurs during pre-launch testing or mid-flight.


---
# System Design Justification — Consolidated Engineering Rationale


- SX1276 LoRa Transceiver
    - Long-range, low-power, high interference immunity — well-suited to shared launch sites with multiple concurrent RF systems

- SF7 Spreading Factor
    - Highest LoRa throughput; 56 dB link budget margin makes additional margin from higher SF unnecessary

- Dual-Path Architecture
    - Simultaneous LoRa telemetry + microSD logging — mission data survives any link failure; no in-flight recovery possible otherwise

- Configurable FSM Thresholds
    - Thresholds beyond the mission-specified 600 m AGL are deferred — separates tuning from firmware recompilation, enabling rapid post-test iteration

- Layered Validation Architecture
    - 4 independent layers (PHY sync, team ID, CRC, XOR) each address a distinct failure mode — no single point of validation failure

- Vertical Antenna Polarisation
    - Matched polarisation on both ends eliminates 3–6 dB of avoidable mismatch loss

- Omnidirectional Antenna Pattern
    - CanSat rotates continuously during descent — omnidirectional pattern avoids orientation-dependent signal nulls a directional antenna would introduce

- Higher-Gain GCS Antenna
    - 5–8 dBi collinear compensates for CanSat's constrained monopole and boosts link margin without CanSat-side complexity or tracking hardware



---
# Preliminary Link Budget Analysis


- Assumptions
    - TX Power (Pt) = 17 dBm
    - CanSat Antenna Gain (Gt) = 2 dBi 
    - GCS Antenna Gain (Gr) = 5 dBi
    - Link Distance (d) = 1 km | Frequency (f) = 867 MHz
    - SX1276 Sensitivity (SF7, 125 kHz BW) = −123 dBm

- Equations
    - FSPL(dB) = 20·log₁₀(d) + 20·log₁₀(f) + 32.44 Pr = Pt + Gt + Gr − FSPL
    - LM = Pr − Sensitivity

- Calculations
    - FSPL = 20·log(1) + 20·log(867) + 32.44 ≈ 91 dB 
    - Pr = 17 + 2 + 5 − 91 = −67 dBm
    - LM = −67 − (−123) = 56 dB

- Results and Interpretation
    - FSPL @ 1 km, 867 MHz ≈ 91 dB
    - Received Power (Pr) ≈ −67 dBm
    - Link Margin (LM) ≈  56 dB — ROBUST

- Significance of 56 dB Margin
    - The received signal is 56 dB above the SX1276 demodulation floor. Each 6 dB of margin doubles the tolerable signal degradation, so 56 dB represents a very large safety factor.
    - This margin accommodates:
        - Antenna orientation losses: 10–15 dB for a misaligned monopole — fully covered
        - Typical multipath fading: 10–20 dB — fully covered
        - Launch-site RF interference from co-located teams — covered
    - SF7 is confirmed appropriate for the 1 km LOS mission profile, with margin to adjust to a higher spreading factor if ground testing reveals degradation not captured by the free-space model.


---
# Mission Timing Architecture — GNSS-Synchronised


- No Dedicated RTC Module
    - The CDH subsystem does not use a dedicated external real-time clock (RTC) IC. Instead, mission timing is implemented as a GNSS-synchronised architecture.

- Time Architecture
    - GNSS UTC time (NEO-M9N) is the primary absolute time reference 
    - STM32F405 generates mission elapsed time relative to launch epoch 
    - Mission time included in every telemetry packet (offset 2)
    - Mission time persisted to microSD and restored on reset recovery 
    - Raw GNSS time (offset 24) also logged for post-flight correlation

- Why GNSS-Synchronised vs. RTC?
    - A GNSS-derived absolute reference is referenced to a global atomic time standard, eliminating the drift associated with onboard crystal oscillators over multi-hour missions — without the added component, power, and battery-backup overhead of a separate RTC IC.

- Timing Data Flow
    - NEO-M9N GNSS Receiver:            Provides UTC time — primary absolute reference
    - STM32F405 Flight Computer:        Generates mission elapsed time from launch epoch
    - Telemetry Packet (offset 2):      mission_time included in every 75-byte packet
    - microSD Persistence:              mission_time saved each cycle — reset recovery source
    - Telemetry Packet (offset 24):     Raw gnss_time also logged for post-flight correlation

- Dual recording of both GNSS time and mission elapsed time in every packet allows post-flight analysis to cross-reference onboard event timing against an independent absolute clock.



---
# Transmission Loss Mitigation and Backup RF Capability


- Dual-Path Telemetry Handling
    - Primary — Real-Time Telemetry:                SX1276 LoRa downlink to GCS at 1 Hz — live mission monitoring
    - Secondary — Onboard Data Preservation:        Simultaneous logging of the same packet to FAT32 microSD, independent of RF link status

- Key Outcome
    - Every telemetry packet generated by the flight computer is both transmitted via LoRa and logged to microSD. Because both paths execute for every packet, loss of the RF communication link does not result in loss of mission data — all mission telemetry remains available for post-flight recovery and analysis via the onboard storage subsystem.

- PDR-Stage Mitigation Approach
    - Transmission-loss mitigation at PDR stage is achieved through dual-path telemetry handling — real-time LoRa transmission combined with simultaneous onboard logging — rather than through a currently implemented secondary RF link.

- Backup RF Channel Capability
    - CMD_SWITCH_CHANNEL (Opcode 0x08)
    - Reserved capability for alternate RF channel selection — implemented and validated through the standard telecommand pipeline

- Deferred to CDR — Reasons
    - Final spectrum planning has not yet been completed Channel allocation depends on integrated RF testing
    - Current PDR focuses on the primary communication architecture

- Backup During Transmission Loss — SATISFIED
    - Via simultaneous onboard logging + mission-state persistence. Backup RF channel capability exists in command architecture; channel definitions remain a CDR item
