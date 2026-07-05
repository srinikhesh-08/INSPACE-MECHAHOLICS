These are the EPS slides from our PDR
##CANSAT COMPONENT SUMMARY
| Category | Component | Function / Role | Operating Voltage |
|----------|-----------|-----------------|-------------------|
| Flight Computer | STM32F405 | Main MCU – telemetry, sensor fusion, mission control | 5 V |
| Altimeter | BMP585 | High-precision barometric altimeter and temperature sensor | 3.3 V |
| IMU | BNO086 | 9-axis orientation, acceleration, and altitude estimation | 3.3 V |
| GNSS | NEO-M9N | Quad-constellation positioning and velocity | 5 V |
| Telemetry Radio | SX1276 LoRa | Long-range RF communication (868 MHz) | 3.3 V |
| Camera | M5Stack CAM (OV2640) | Mission imaging and video capture | 5 V |
| Data Logger | MicroSD Module | SPI flight data logging | 3.3 V |
| Buck Converter | MP1584 | Converts 7.4 V to 5 V, up to 3 A, 90–92% efficiency | Input: 7.4 V |
| LDO Regulator | TPS73733 | Converts 5 V to 3.3 V, 1 A, low-noise, high-PSRR | Input: 5 V |
| Surge Protection | SMBJ12A TVS Diode | Transient Voltage clamp-12V, 600 W peak | - |
| Voltage measurement | R-Divider 100k/47k+ADC | Battery voltage → STM32 PA0 ADC | - |
| Power Control | External Toggle Switch | Accessible main power ON/OFF | - |
| Power Indicator | Green LED+330Ω | Power ON visual indicator→PC13 | 3.3V |

##EPS ARCHITECTURE
| Parameter | Specification |
|-----------|---------------|
| **Battery** | Samsung INR18650-30Q, 2S1P configuration, 7.4 V, 3000 mAh |
| **Main 5 V Rail** | MP1584 Buck Converter: 7.4 V → 5 V, 3 A max, ~90% efficiency |
| **Logic 3.3 V Rail** | TPS73733 LDO Regulator: 5 V → 3.3 V, 1 A, low-noise output |
| **Power Architecture** | Dual regulated voltage rails with centralized star power distribution |
| **Protection** | Fuse, MOSFET reverse-polarity protection, and TVS surge clamp |
| **Mission Runtime** | Minimum requirement: 2 hours • Estimated operational time: ~7 hours |
| **Total Energy** | 22.2 Wh available • 5.64 Wh required (includes 30% design margin) |

##PAYLOAD BLOCK DIAGRAM
Samsung INR18650-30Q
        |
        |
  Toggle switch
        |
MP1584 Buck Converter: 7.4 V → 5 V ----TPS73733 LDO Regulator: 5 V → 3.3 V
        |                                              |__ BMP585
        |__ STM32F405RGT6                              |__ BNO086
        |__ NEO-M9N GPS                                |__ MicroSD card
        |__ M5Stack CAM                                |__ SX1276 LoRa

## Battery Comparison

| Parameter | Samsung INR18650-30Q | Panasonic NCR18650GA | Molicel INR21700-P45B |
|-----------|----------------------|----------------------|-----------------------|
| Cell Type | Li-Ion INR18650 | Li-Ion NCR18650 | Li-Ion INR21700 |
| Nominal Voltage | 3.6 V | 3.6 V | 3.6 V |
| Full Charge Voltage | 4.2 V | 4.2 V | 4.2 V |
| Capacity | 3000 mAh | 3500 mAh | 4500 mAh |
| Energy per Cell | 10.8 Wh | 12.6 Wh | 16.2 Wh |
| Maximum Continuous Current | 15 A | 10 A | 45 A |
| Weight per Cell | 48 g | 48 g | 70 g |
| Dimensions | 18.4 × 65 mm | 18.6 × 65 mm | 21 × 70 mm |
| Thermal Stability | Very Good | Excellent | Excellent |
| Cost per Cell | ₹550–650 | ₹700–850 | ₹1000–1200 |

## Battery Configuration Selection

| Configuration | Nominal Voltage | Capacity | Power Flow | Max Discharge Power | Verdict |
|---------------|----------------:|---------:|------------|--------------------:|:-------:|
| 1S Single Cell | 3.7 V | 3000 mAh | 3.7 V → Boost → 5 V → LDO → 3.3 V | 9.25 W | Rejected |
| 2S1P Series | 7.4 V | 3000 mAh | 7.4 V → Buck → 5 V → LDO → 3.3 V | 22.2 W | Selected |
| 2P Parallel | 3.7 V | 6000 mAh | 3.7 V → Boost → 5 V → LDO → 3.3 V | 18.5 W | Rejected |

## Power Budget Estimation

| Type | Component | Voltage (V) | Current (mA) | Power (W) | Duty Cycle (%) | Energy (2h) (Wh) | Source |
|------|-----------|-------------|--------------|-----------|----------------|------------------|--------|
| MCU | STM32F405 | 3.3 | 110 | 0.363 | 100 | 0.726 | Datasheet |
| Sensor | BMP585 | 3.3 | 0.005 | 0.000016 | 100 | 0.00003 | Datasheet |
| IMU | BNO086 | 3.3 | 18 | 0.059 | 100 | 0.118 | Datasheet |
| GPS | NEO-M9N | 3.3 | 68 | 0.224 | 100 | 0.448 | Datasheet |
| Radio TX | SX1276 LoRa (TX) | 3.3 | 120 | 0.396 | 20 | 0.158 | Datasheet |
| Radio Idle | SX1276 LoRa (Idle) | 3.3 | 10 | 0.033 | 80 | 0.053 | Datasheet |
| Storage | microSD Module | 3.3 | 50 | 0.165 | 100 | 0.330 | Datasheet |
| Camera | M5Stack CAM | 5.0 | 160 | 0.800 | 100 | 1.600 | Datasheet |
| LED | Power LED | 3.3 | 10 | 0.033 | 100 | 0.066 | Estimated |
| Losses | Regulators & Conversion | — | — | 0.420 | 100 | 0.840 | Calculated |
| **TOTAL** |  |  |  | **2.49 W** |  | **4.339 Wh** |  |

## Battery Pack Energy & Mission Analysis

| Parameter | Value |
|----------|------|
| Total Available Energy | 22.2 Wh |
| Average Power Consumption | 2.49 W |
| Mission Energy (2 hours) | 4.34 Wh |
| Required Energy (with 30% margin) | 6.47 Wh |
| Battery Voltage | 7.4 V (2S) |
| Capacity | 3000 mAh |
| Safe Runtime | ~7 hours |
| Theoretical Runtime | 8.91 hours |
| Energy Remaining | 15.73 Wh |
| Status | ✅ COMPLIANT |

---

### Energy Calculations

**Battery Energy**
```
Energy = Voltage × Capacity
       = 7.4 V × 3.0 Ah
       = 22.2 Wh
```

**Mission Energy (2 hours)**
```
= 2.49 W × 2 h
= 4.98 Wh
```

**With 30% Safety Margin**
```
= 4.98 Wh × 1.3
= 6.47 Wh
```

**Energy Margin**
```
= 22.2 Wh − 6.47 Wh
= 15.73 Wh
```

**Operating Time**
```
= 22.2 Wh / 2.49 W
= 8.91 hours (theoretical)
```

---

### Notes
- Safe runtime is derated to ~7 hours due to:
  - temperature effects  
  - battery aging  
  - RF transmission bursts  
- System is comfortably above the 2-hour competition requirement with strong margin.

