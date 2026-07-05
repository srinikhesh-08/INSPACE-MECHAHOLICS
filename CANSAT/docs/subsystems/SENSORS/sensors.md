## Sensor Subsystem Breakdown

| Subsystem | Primary Sensor | Function |
|------------|----------------|----------|
| Altitude + Temperature | BMP581 | Provides integrated pressure and temperature readings, eliminating the need for a separate temperature module |
| Air Temperature | BMP581 (Integrated) | Measures ambient air temperature as part of barometric sensing |
| Navigation (GNSS) | NEO-M9N | Measures absolute position, velocity, and time data throughout flight |
| Tilt Orientation / IMU | BNO086 | Tracks orientation, rotation rate, and linear acceleration |
| Magnetometer | BNO086 (via fusion) | Provides Earth magnetic field direction and magnitude using onboard sensor fusion (CEVA) |
| Telemetry | LoRa SX1278 | RF communication at 433 MHz for long-range data transmission |
| Data Logging | microSD (SPI) | Onboard storage of flight data for post-mission analysis |

## Barometric Sensor Comparison

| Sensor | Weight | Pressure Accuracy | Altitude Accuracy | Communication Interface | Output Data Rate (ODR) | Power Consumption | Operating Voltage | Cost (INR) | 
|--------|--------|------------------|-------------------|--------------------------|--------------------------|------------------|-------------------|------------|
| BMP180 | 5 g | ±1–4 hPa | ~0.03 hPa (~varies, 18-bit resolution) | I2C | ~1 Hz | ~650 µA | 1.8–3.6 V | ~₹50 |
| BMP280 | 3–5 g | ±1 hPa @ 25°C | ±0.12 hPa (~±1 m) | I2C, SPI | Moderate | ~714 µA | 1.71–3.6 V | ₹400–800 |
| BMP581 / BMP585 | 3–5 g | ±0.5 hPa max | ±6 Pa (~±0.5 m) | I2C, I3C, SPI | Up to 622 Hz | ~260 µA (active) | 1.71–3.6 V | ₹800–1100 |

**Critical Performance Requirements** 
• Altitude resolution: < 1 m  (required for apogee detection) 
• Sampling rate: 100–622 Hz  (fast ascent tracking) 
• Noise immunity: Critical  (high vibration environment) 
• Shock survival: Passive MEMS OK with foam/rubber damping 
• Temperature: Onboard compensation preferred for flight accuracy 

## Payload Air Temperature selection

| Role | Sensor | Justification |
|------|--------|---------------|
| Primary Altimeter | BMP581 / BMP585 | 622 Hz ODR, ±0.5 m accuracy, FIFO buffer, onboard low-pass filtering — best-in-class performance for rocketry applications |
| Backup Altimeter | BMP280 | Proven, low-cost redundancy with sufficient accuracy for fail-safe altitude estimation |
| Temperature Sensor | BMP581 (Integrated) | Uses internal calibrated temperature sensor — reduces weight, cost, and system complexity |

## GNSS Module Comparison

| Sensor | Weight | Position Accuracy | Satellite Constellations | Communication Interface | Output Data Rate (ODR) | Power Consumption | Operating Voltage | Cost (INR) |
|--------|--------|------------------|---------------------------|--------------------------|--------------------------|------------------|-------------------|------------|
| NEO-6M | ~50 g | ~2.5 m | GPS only | UART | ~1 Hz (low) | Low | 2.7–3.6 V | ₹300–500 |
| NEO-M8N | ~25 g | ~0.6–0.9 m | GPS + GLONASS | UART | Moderate | Low | 3.3 V | ~₹2000 |
| NEO-M9N | Light | ~1.5 m CEP | GPS + GLONASS + Galileo + BeiDou | UART, I2C, SPI | Up to 20–30 Hz (varies by config) | ~30 µA (low-power mode) | 3.3 V | ₹2000+ |

**Sensor Justification:**
**Primary**- GPS NEO-M9N:
25 Hz, quad-constellation, anti jamming, 80 km altitude, 500 m/s 
— rated for aerospace dynamics
**Secondary Backup**- GPS NEO-M8N
High accuracy (0.9 m), proven in UAVs/drones, lower cost fallback

## IMU Sensor Comparison

| Sensor | Accel Range | Sensor Fusion | Output Data Format | Communication Interface | Gyro Range | Power Consumption | Operating Voltage | Cost (INR) |
|--------|-------------|---------------|---------------------|--------------------------|-------------|------------------|-------------------|------------|
| MPU-6050 | ±2–16 g | DMP (built-in) | Raw + DMP | I2C | ±250–2000 dps | Low | 2.3–3.4 V | ₹150–250 |
| BNO055 | ±2–16 g | Built-in fusion | Euler, Quaternion | I2C | ±250–2000 dps | Low | 2.4–16 V | ₹1300–1700 |
| BNO086 | ±2–16 g | Advanced Motion Engine | Orientation, rotation vectors | I2C, SPI | High precision | Moderate (~30 µA) | 3.3 V | ₹2000+ |

**BNO086-**
• CEVA Motion Engine: advanced onboard sensor fusion 
• Real-time orientation output with dynamic calibration 
• Handles vibration and high-motion environments better than BNO055 
• Minimizes MCU load — direct quaternion/rotation vector output 
• Designed for robotics, drones, and high-motion aerospace systems

## Final Sensor Combination

| Sensor Role | Sensor | Justification |
|-------------|--------|---------------|
| Primary Orientation | BNO086 (IMU) | Sensor fusion IMU eliminates magnetometer noise issues; best practical solution for stable real-time orientation |
| Backup Magnetometer | MMC5983MA | Used only if implementing advanced external fusion; high precision (±0.5°) with up to 1000 Hz ODR |
| Temperature | BMP581 (Integrated) | Uses internal calibrated temperature sensor — no extra weight, cost, or complexity |
