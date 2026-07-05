## Mechanical Subsystem

The Mechanical Subsystem is responsible for:

- Providing the structural framework of the CanSat
- Ensuring safe integration of all components
- Supporting the recovery system

Major Structural Components

1. Outer Casing
2. Structural Frame
3. Plates

### Structural Components
#### Outer Casing

Cylindrical structure conforming to CAN-7USAT dimensions (In our case it is 15x40 CM)
Protects internal components (electronics) from dust and moisture in air and also protects from Mechanical shocks during ascent and descent 
Design Requirements:
- Lightweight 
- Accessible 
- Ventilation 
CanSat outer casing should be light in weight so that it meets the mass budget but, should be strong enough and should have access to remove top and bottom for better integration and ventilation should be provided through vent holes so that sensors like pressure,temperatures exposed to outside air 

#### Materials for outer casing

- PETG
- PLA
- ABS
- Fiberglass
- Carbon fiber: By using 3D-printing we could go with PETG as better option because it is strong and light in weight compared to others


#### Structural frame 
    It is the skeleton of the cansat .It supports all components to maintain their position and protect from loads experienced during the launch,payload separation and landing.
    
#### Functions:
Components mounting 
Absorbs loads
Protection and alignment 

#### Frame Design

Number of Pillars: 6

A **hexapillar configuration** has been selected instead of 3-pillar or 4-pillar designs.

Reasons for Selection

- Higher structural strength
- Better vibration resistance
- Greater torsional rigidity

#### Materials

- 3D-printed PETG circular plates
- Aluminum pillars

Estimated Mass

- Structural frame mass: **200–250 g**

---

###  Structural Components

#### Plates

Plates serve as mounting surfaces for:

- Provides mounting surfaces for PCBs,sensors,batteries.
- Increase stiffness for CanSat 
- They are Manufactured from Lightweight material either aluminum or composites

 Other things such as fasteners and connectors are stainless steel screws ,etc..
 Safe integration of components in these structures provides nominal performance of components which meets the mission requirements such as recovery, telemetry,etc..


#### Recovery System
After separation from Launcher, CanSat needs it’s own recovery system to land safely without any damages . CanSat will have two descent control systems( Primary and Secondary) 
Recovery options:                                                           
- Parachutes 
- Rotors 
- Gliders 
- Streamers 
 The best options among them  are either parachutes or rotors, glider and streamers are not preferable due to mass and volume constraints 

#### Parachutes deploying mechanism :
Parachutes and spring system are well packed and prepared for launch, Once CanSat is deployed from the rocket, flight controller detect the condition and send command to servo to rotate and latch disengages so that spring push the top part of the CanSat and parachutes will be expelled into airflow and it inflates and continues controlled descent till landing

For **primary descent control** we use drogue parachutes will 
Engage at apogee while for secondary descent control system

We use main parachutes which will engage at 600m altitude 

#### Backup descent strategy:
All the conditions will updated to MCU by altitude sensor so, in recovery it plays a vital in engaging of parachutes at desired altitudes. In case, if any anomaly in sensor part then we activate COTS altimeter which is commercial altimeter, which will monitor the altitude independently and will guide the MCU to take further actions to recover the CanSat by deploying parachutes. 

##### COTS Preference Order:                        
1. Missile Works RRC3+
2. PerfectFlite StratoLoggerCF
3. Eggtimer Quark

---

### Parachutes: 
#### Aerodynamics of Parachutes:
The drag force is dependant upon 
1) the dynamic pressure created by moving air striking the parachute canopy (and which keeps the parachute inflated)
2) the diameter of the parachute, which determines the area over which the dynamic pressure acts 
3) the drag coefficient, Cd, of the parachute.
 Dynamic pressure is a function of velocity and air density, which in turn is dependent upon altitude and temperature.

####  Descent rate of parachutes
The vertical descent rate provided by parachute in stable descent rate is give by :

<img width="95" height="47" alt="image" src="https://github.com/user-attachments/assets/80bfae67-cc78-49ce-be6d-352adc63a911" />

Wt = total weight of body and parachute 
S = canopy reference (surface) area   
r = air density 
To get the canopy area of parachutes :
To get descent velocity around 5m/s the relation between diameter and mass is given by:  
                                                Diameter = 0.9*(mass)^½  where mass is in kg  
                                  **Since our CanSat mass is around 1kg , required diameter is around 90cm for main chutes**

####  Calculations for descent velocity 
Mass m=1kg
Parachute diameter d=0.9m
Area A=πr² =0.64m² 
Drag coefficient Cd=0.8
Air density ρ=1.225 kg/m³ 
Using:
v=√ (2mg/ρCdA)  ≈  5.6 m/s
The terminal descent velocity is subjected to change based on air density at location of launch and weather conditions 

#### Parachute Design
Types of parachutes based on the shapes:
More common shapes are flat circular, conical, bi-conical, hemispherical, ellipsoidal, cross, annular, toroidal, ringslot, ringsail and ribbon- type 
1. Parasheet or flat circular                                          
2. Elliptical or ellipsoidal
3. Cruciform or cross

The parasheet is simplest to make, it is basically a single piece of fabric cut to a circular shape. Shroud lines are sewn onto the hemmed circumference.

####  Parachute design 

<img width="203" height="86" alt="image" src="https://github.com/user-attachments/assets/bc767fdc-5d90-4c12-b855-89b2ea04c0a9" />
<img width="149" height="106" alt="image" src="https://github.com/user-attachments/assets/709be172-1673-4b39-9589-7c160d2a3fa1" />

According to some research papers,
Circular form parachute is considered best of these        
1. It will give stable descent rate 
2. Prevent swinging(oscillations)
3. Open reliably 
Material: Ripston Nylon           

---

### Mass Budget

#### Electrical Components
 
| Component | Model | Est. Mass (g) |
|-----------|-------|---------------:|
| Flight Computer | STM32F405 Flight Controller | 15 |
| Altimeter | BMP585 Breakout Module | 20 |
| IMU | BNO086 Breakout Module | 3 |
| GNSS | NEO-M9N Module + antenna | 22 |
| Telemetry Radio | SX1276 LoRa Module | 5 |
| Camera | M5StackCAM (OV2640) | 15 |
| Data Logger | MicroSD Module + Card | 4 |
| Buck Converter | MP1584 Module | 10 |
| LDO Regulator | TPS73733 Circuit | 1 |
| Wiring & Connectors | JSTs, headers, cables | 30 |
| PCB (custom avionics board) | Main electronics board | 30 |
| **Total (Electrical)** | | **155** |
 
#### Mechanical Components
 
| Component | Est. Mass (g) |
|-----------|---------------:|
| 2× Samsung INR18650-30Q | 100 |
| Primary Parachute | 25 |
| Secondary Parachute | 15 |
| Servo Mechanism | 30 |
| CanSat Structure (frame, fasteners, mounts) | 300 |
| Recovery Buzzer | 10 |
| Status LEDs | 2 |
| **Total (Mechanical)** | **482** |
 
#### Total Mass Budget
 
```
Total Mass = Electrical (155 g) + Mechanical (482 g) = 637 g
```
