

# FSW Overview

- The finite state machine contains the above states:
    - BOOT
        - Hardware &software initialisation.Sensor buses, SDcard mount, previous state recovery, health diagnostics.
    - TEST_MODE
        - Live sensor validation, telemetry demo packets, SD write test, GCS link verification before launch.
    - LAUNCH_PAD
        - Pre-launch idle. 1 Hz telemetry begins. Gyro, baro & accel calibration on command. Launch detection armed.
    - ASCENT
        - All telemetry streaming.Continuous altitude, IMU, GNSS, voltage logging. Apogee detection via altitude trend.
    - PRIMARY_DESCENT
        - Post-ejection. Primary parachute deployed. Passive stabilisation. Descent rate target <20 m/s.
    - SECONDARY_DESCENT
        - Activated at 600 ± 10 m AGL. Secondary mechanism deployed.Gyroscopic stabilisation active. Target 1–3 m/s.
    - IMPACT
        - Low altitude + IMU deceleration spike. Audio beacon on. Final telemetry sent. SD card finalised.

- The FSW was divided into the following modules by functionality:
   
                          +------------------------------------------------------+
                          |                Communication Layer                   |
                          | GCS uplink/downlink • Packet serialization • Commands|
                          +--------------------------+---------------------------+
                                                     |
                                                     v
                          +------------------------------------------------------+
                          |                     FSW Core                         |
                          | State Machine • Watchdog • Task Scheduler • Commands |
                          +--------------------------+---------------------------+
                                                     |
                                                     v
                          +------------------------------------------------------+
                          |                    Data Layer                        |
                          | Sensor Fusion • Complementary Filter • AbstractState |
                          +--------------------------+---------------------------+
                                                     |
                           +-------------------------+-------------------------+
                           |                                                   |
                           v                                                   v
          +--------------------------------------+      +--------------------------------------+
          |      Hardware Controller Layer       |      |         TestAndSim Suite            |
          | IMU • Barometer • GNSS • LoRa • SD   |      | Drop-in Hardware Replacement        |
          | Hardware Drivers & Interfaces        |      | Simulated Sensors & Testing         |
          +--------------------------------------+      +--------------------------------------+

                                          
                                          
                        +----------------------------------------------+
                        |           Config Module (config.h)           |
                        | Shared configuration used by all layers      |
                        +----------------------------------------------+
```

---


- FSW TASKS (Scheduled by FreeRTOS)
    - TaskIMU
    - TaskBaro
    - TaskGNSS
    - TaskDataLayer
    - TaskFSM
    - TaskTelemetry
    - TaskCommRx
    - TaskSDWrite
    - TaskHealthMonitor
    - TaskRecoveryBeacon
- Platform
    - MCU: Arduino Nano 33 BLE Sense Rev2
    - ARM Cortex-M4F @ 64 MHz · C++ / PlatformIO + VSCode
    - Radio: XBee PRO S3B 900 MHz
    - NETID/PANID = 3165 · 1 Hz telemetry downlink
    - Storage: SD card + processor flash memory
    - State and telemetry persisted across resets
    - RTC: DS3231 mini
    - Maintains mission time through power loss

---

# Task Sampling Rates Across FSM States

```text
+-------------------+--------------------+-------+------+-------------+--------+------------------+--------------------+----------+
| Task              | Module             | BOOT  | TEST | LAUNCH_PAD  | ASCENT| PRIMARY_DESCENT | SECONDARY_DESCENT | IMPACT   |
+-------------------+--------------------+-------+------+-------------+--------+------------------+--------------------+----------+
| TaskIMU           | HardwareController | 10 Hz |10 Hz | 10 Hz       |100 Hz | 50 Hz           |100 Hz             |10 Hz     |
| TaskBaro          | HardwareController |  5 Hz | 5 Hz |  5 Hz       |20 Hz  | 10 Hz           |20 Hz              | 2 Hz     |
| TaskGNSS          | HardwareController |  1 Hz | 1 Hz |  5 Hz       | 5 Hz  |  5 Hz           | 5 Hz              | 1 Hz     |
| TaskDataLayer     | DataLayer          |  5 Hz | 5 Hz |  5 Hz       |20 Hz  | 10 Hz           |20 Hz              | 2 Hz     |
+-------------------+--------------------+-------+------+-------------+--------+------------------+--------------------+----------+
| TaskFSM           | FSWCore            | 10 Hz |10 Hz | 10 Hz       |10 Hz  | 10 Hz           |10 Hz              |10 Hz     |
| TaskHealthMonitor | FSWCore            |  1 Hz | 1 Hz |  1 Hz       | 1 Hz  |  1 Hz           | 1 Hz              | 1 Hz     |
+-------------------+--------------------+-------+------+-------------+--------+------------------+--------------------+----------+
| TaskTelemetry     | CommLayer          | OFF   | OFF  | 1 Hz*       |1 Hz   | 1 Hz            |1 Hz               |Final TX  |
| TaskCommRx        | CommLayer          | OFF   | OFF  | Interrupt   |Interrupt | Interrupt     |Interrupt          |OFF       |
+-------------------+--------------------+-------+------+-------------+--------+------------------+--------------------+----------+
| TaskSDWrite       | HardwareController | 1 Hz  |1 Hz  | 1 Hz        |1 Hz   | 1 Hz            |1 Hz               |Finalize  |
| TaskComm4G        | HardwareController |Dormant|Dormant| Dormant    |Dormant| Dormant         |Dormant            |Once      |
+-------------------+--------------------+-------+------+-------------+--------+------------------+--------------------+----------+

Notes:
* TaskTelemetry starts after CMD_START_TELEMETRY (expected during LAUNCH_PAD).
* TaskFSM and TaskHealthMonitor maintain a constant execution rate.
* Periodic task rates are updated by the FSW Core on state transitions.
* TaskCommRx is interrupt-driven (SX1278 RX ISR), not periodically scheduled.
```

---



# Payload FSW State Diagram (1/2)

## Payload FSW State Machine

```text
                               +--------------------------------------------------+
                               |                     BOOT                         |
                               | Init sensors • Mount SD • load previous state    |
                               | Health diagnostics                               |
                               +------------------------+-------------------------+
                                                        |
                                                        | All subsystems initialized
                                                        v
                               +--------------------------------------------------+
                               |                  TEST_MODE                       |
                               | Live sensor check • Telemetry demo               |
                               | SD test • GCS link verification                  |
                               +------------------------+-------------------------+
                                                        |
                                                        | CMD_ENTER_LAUNCHPAD
                                                        v
                               +--------------------------------------------------+
                               |                 LAUNCH_PAD                       |
                               | 1 Hz telemetry • Calibration commands            |
                               | Launch detection armed                           |
                               +------------------------+-------------------------+
                                                        |
                                                        | Acceleration > Threshold
                                                        | AND Altitude Increasing
                                                        v
                               +--------------------------------------------------+
                               |                   ASCENT                         |
                               | Full telemetry • IMU • GNSS • Voltage Logging    |
                               | Apogee detection                                 |
                               +------------------------+-------------------------+
                                                        |
                                                        | Sustained Negative
                                                        | Vertical Velocity
                                                        v
                               +--------------------------------------------------+
                               |              PRIMARY_DESCENT                     |
                               | Primary parachute deployed                       |
                               | Passive stabilization (<20 m/s)                  |
                               +------------------------+-------------------------+
                                                        |
                                                        | Altitude ≤ 600 ±10 m AGL
                                                        v
                               +--------------------------------------------------+
                               |             SECONDARY_DESCENT                    |
                               | Secondary mechanism deployed                     |
                               | Gyro stabilization active (1–3 m/s)              |
                               +------------------------+-------------------------+
                                                        |
                                                        | Near Ground +
                                                        | IMU Deceleration
                                                        v
                               +--------------------------------------------------+
                               |                   IMPACT                         |
                               | Audio Beacon ON • Final Telemetry                |
                               | SD Finalized • 4G GPS post                       |
                               +--------------------------------------------------+
```
- Onboard sensor suite
    - BMP586 — pressure / altitude / temperature (I²C/SPI)
    - NEO-M9N — GNSS position, altitude, UTC (UART/I²C/SPI)
    - MPU6050 — acceleration, gyroscope (I²C)
    - BNO085 — orientation, magnetometer (UART/I²C/SPI)
    - M5Stack Unit Cam- OV2640 (UART/WiFi)
    - Voltage monitor — system power health (ADC)

---

# Simulation Mode Software

- TestAndSim suite - overview
    - A compile-time swappable module (activated via config.h flag) that replaces the Hardware Controller Layer with procedurally generated sensor data. All upper layers i.e FSW Core, Data Layer and CommLayer; execute identically to flight firmware. LoRa is replaced by a loopback stub for packet inspection without radio hardware. The actual data generated is dependent on the current preset scenarios of the mission we will create, and will be saved as a collection of `scenarios` such as Nominal Flight, Apogee Edge Case, Launch Edge case and others.
- SimDriver_t controller struct
    - Owns: simulated sensor readings per channel, active scenario ID, simulation tick counter, FaultFlags_e for fault injection. Tick rate matches the highest active sensor acquisition rate to exercise full FreeRTOS load.
- Noise model
    - Per-channel noise consistent with published sensor datasheets. Fixed seed for deterministic regression runs; randomised seed for stress testing. Both modes selectable via SimDriver_t config field.
    

---

# Test Methodology

- The TestAndSim suite also will contain an extensive testing suite that uses both white-box and black-box testing to ensure the robustness of the software.
    - Two-level testing: unit tests (white-box) + full pipeline tests (integration, black-box) via TestAndSim; hardware testing covered separately
    - White-box: checks individual functions (FSM logic, sensor fusion, packet formatting, SD card handling) in isolation, including edge cases
    - Black-box: runs full simulated flight scenarios and checks telemetry output and state transitions against expected behavior
    - Some checks: correct state sequencing, packet counter continuity, threshold/boundary behavior, fault handling without system stalls, recovery after reset.
    - All thresholds are pulled from a central config file, so tests auto-update when values are tuned
    - Any failed test blocks are caught during development so issues are caught early, before they compound.


---

# Software Development Plan(1/2)

- Prototyping & prototyping environments
    - Development host (unit testing): White-box unit tests run on the development PC without FreeRTOS or physical hardware. Enables rapid iteration during early firmware development.
    - TestAndSim environment (integration): Full FSW pipeline on STM32 with TestAndSim replacing Hardware Controller Layer. LoRa replaced by loopback stub.
    - Proto hardware bench: Assembled proto unit with all sensors soldered and bus-connected. Sensor bring-up per sensor type
- Test Methodology
    - All tests versioned alongside firmware source. Results must be stable across multiple runs with differing noise seeds
    - Sensors are tested independently and are then integrated with FSW

    ```text
- Software subsystem development sequence

   ## Software Subsystem Development Sequence

```text
+------------------------+
| Phase 1                |
| Config & Types         |
+------------------------+
| • config.h             |
| • types.h              |
| • Thresholds           |
+------------------------+
            │
            ▼
+------------------------+
| Phase 2                |
| Hardware Controller    |
| Layer                  |
+------------------------+
| • Sensor Drivers       |
| • LoRa Tx/Rx           |
| • SD Submodule         |
+------------------------+
            │
            ▼
+------------------------+
| Phase 3                |
| Data Layer             |
+------------------------+
| • Complementary Filter |
| • Sensor Fusion        |
| • AbstractState        |
+------------------------+
            │
            ▼
+------------------------+
| Phase 4                |
| FSW Core & FSM         |
+------------------------+
| • 7-State FSM          |
| • Watchdog             |
| • State Persistence    |
+------------------------+
            │
            ▼
+------------------------+
| Phase 5                |
| Communication &        |
| Storage                |
+------------------------+
| • Binary Telemetry     |
| • Command Dispatch     |
| • SD Write Logic       |
+------------------------+
            │
            ▼
+------------------------+
| Phase 6                |
| TestAndSim &           |
| Integration            |
+------------------------+
| • SimDriver_t          |
| • Mission Scenarios    |
| • Full Pipeline Tests  |
+------------------------+
```
    ```

- Key development dependencies & milestones
    - Phase 1 (config.h / types.h) is the single source of truth and must be locked before Phase 5 binary serialisation
    - Phase 2 sensor drivers are independently testable on bench; bring up one sensor per session
    - Phase 3 complementary filter validated against synthetic inputs (unit test) before integrating with real sensors
    -  Phase 4 FSM gate: all 7 states must traverse correctly in TestAndSim nominal scenario before Phase 6 begins
    -  Phase 6 regression baseline (nominal flight scenario) becomes mandatory CI gate — failure blocks further testing Hardware proto validation (drop test, LoRa range, watchdog timeout) run in parallel with Phase 6 integration

