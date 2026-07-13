# INSPACE-MECHAHOLICS
## This is where all the hardware and software documentation for the competition will be saved

## MECHANICAL SUBSYSTEM

The Mechanical Subsystem provides the structural framework, component integration, and recovery system for the CanSat. The design follows the CAN7USAT dimensional constraints (15 cm × 40 cm) while maintaining low mass and high structural integrity during launch, deployment, descent, and landing.

The structure consists of a lightweight PETG outer casing, a six-pillar aluminium frame, and internal mounting plates for electronics, batteries, and sensors. The six-pillar configuration was selected over conventional three- and four-pillar designs due to its improved stiffness, vibration resistance, and torsional rigidity.

Recovery is achieved using a dual-stage parachute system. A drogue parachute is deployed at apogee to stabilize the vehicle, followed by the main parachute at approximately 600 m AGL to achieve a controlled landing. Deployment is servo-actuated using a spring-loaded release mechanism. A commercial off-the-shelf (COTS) altimeter provides redundant deployment capability in the event of primary sensor failure.

## ELECTRICAL POWER SYSTEM

The Electrical Power Subsystem (EPS) supplies regulated power to all avionics while ensuring reliable operation throughout the mission.

The system uses a 2S1P Samsung INR18650-30Q Li-ion battery pack (7.4 V, 3000 mAh), selected for its high current capability, competition compliance, and excellent energy density. Power is regulated through an MP1584 buck converter (7.4 V → 5 V) followed by a TPS73733 low-noise LDO regulator (5 V → 3.3 V), providing dedicated rails for digital logic and sensors.

Protection features include reverse-polarity protection, transient voltage suppression (TVS), fuse protection, battery voltage monitoring, and an externally accessible master power switch.

The average system power consumption is approximately 2.5 W, providing over 7 hours of safe operating time, significantly exceeding the competition's two-hour minimum requirement.

## SENSOR SUBSYSTEM
The Sensor Subsystem collects all mission-critical flight data required for navigation, telemetry, recovery, and post-flight analysis.

The primary altitude and temperature sensor is the BMP581, providing high-resolution pressure measurements and integrated temperature sensing. Vehicle orientation is determined using the BNO086 intelligent IMU, which performs onboard sensor fusion to provide stable quaternion-based attitude estimation while minimizing processor load.

Position and velocity are obtained using the NEO-M9N multi-constellation GNSS receiver, supporting GPS, GLONASS, Galileo, and BeiDou.

Sensor data is transmitted to the ground station through the SX1276 LoRa radio while simultaneously being stored locally on a microSD card for redundancy.

## COMMUNICATION AND DATA HANDLING

The Communication and Data Handling (CDH) subsystem manages onboard data acquisition, storage, telemetry transmission, and communication between all flight hardware.

The STM32F405 flight controller serves as the central processor, collecting data from sensors through I²C, SPI, and UART interfaces. Flight telemetry is transmitted using the SX1276 LoRa radio while identical datasets are simultaneously logged to onboard microSD storage to prevent data loss.

The subsystem maintains synchronized sensor acquisition, packet formation, telemetry scheduling, and storage throughout all flight phases. The communication architecture supports continuous monitoring of system health, mission parameters, and recovery status.

## FLIGHT SOFTWARE

The Flight Software (FSW) controls all mission operations through a finite state machine (FSM) running on FreeRTOS.

Mission execution progresses through seven primary states: Boot, Test Mode, Launch Pad, Ascent, Primary Descent, Secondary Descent, and Impact. Each state activates specific sensing, logging, telemetry, and recovery tasks appropriate for that phase of flight.

The software architecture is task-based, separating sensor acquisition, telemetry, data logging, communication, health monitoring, and state management into independent FreeRTOS tasks. This modular design improves reliability, simplifies debugging, and allows future subsystem expansion.

A dedicated TestAndSim framework enables complete mission simulation using synthetic sensor data, allowing firmware validation without flight hardware. Both unit testing and full mission integration testing are performed before deployment.

## GROUND CONTROL SYSTEM

The Ground Control System (GCS) provides real-time monitoring, visualization, and recording of flight telemetry received from the CanSat.

The desktop application is developed in Python using PyQt6 and PyQtGraph to display live telemetry, mission status, altitude, sensor measurements, and system health. Incoming LoRa packets are decoded, visualized, and simultaneously archived into timestamped CSV files for post-flight analysis.

The GCS also provides mission state indication, communication status, and recovery support, enabling operators to monitor the vehicle throughout the complete mission.
