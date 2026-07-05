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

### LoRa Configuration Protocals
<img width="265" height="133" alt="image" src="https://github.com/user-attachments/assets/ceaf0bdb-7942-456f-a6f6-772087114ff8" />

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
