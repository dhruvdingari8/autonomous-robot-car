# Autonomous Robot Car (ARC)

## 1. Project Overview
This project is a robot car designed and built almost entirely from scratch, including the
3D-printed chassis and all software. Its purpose is to serve as a platform for learning and
experimenting with ROS 2, motor control, control systems, computer vision, etc.

### 1.1. Guiding Principle
Keep version 1 scope in mind, don't let scope creep kill this project. Add features version by version incrementally, rather than all at once.

### 1.2. Version 1 Scope
**Version 1:** the car drives at a commanded velocity, reports its own position through odometry, transmits a camera feed through a webhook, and can be teleoperated from a laptop over ROS 2.

> [!IMPORTANT]
> Out of scope for version 1: object detection, autonomy, and the final 3D-printed chassis.

### 1.3. Long-Term Goals
Two long-term goals influence the version 1 hardware choices. The first is near-full autonomy, using a LiDAR and a camera to map a room (SLAM) and navigate it with Nav2. The second is
two-wheel self-balancing using an IMU. Motors, encoders, and wheels were selected so the same drivetrain can carry over to the balancing configuration.

## 2. System Architecture
### 2.1. Drivetrain
The car uses differential drive in a tricycle layout: two independently driven motors on the rear wheels and a single swivel or ball caster at the front. The robot can turn in place about the
midpoint of the rear axle, with the front caster sweeping a circle around that point. Differential drive was chosen over Ackermann steering for its simpler kinematics, its ability to turn in place,
and its direct support in ROS 2 navigation tools. A two-wheeled balancing robot is also differential drive, so the drivetrain carries over to that goal.

### 2.2. Microcontroller Jobs vs. Raspberry Pi Jobs
Real-time hardware control runs on an ESP32 microcontroller. Kinematics, odometry, ROS 2 communication, and higher-level software run on a Raspberry Pi 5. Linux cannot guarantee timing: a scheduler pause would drop encoder counts (causing permanent odometry drift),
make the PID derivative term jolt and the integral term undercount, and leave the motors uncorrected during the gap. The microcontroller’s hardware timers and interrupts run on schedule, so all timing-critical work lives there. The safety stop must also be on the
microcontroller, because a crashed Pi cannot stop the motors itself. Odometry is integrated on the Pi rather than the microcontroller. This keeps the microcontroller simple, makes the math easy to log and adjust without reflashing, and places odometry
alongside future IMU and camera data for sensor fusion. The microcontroller reports cumulative tick counts, so a lost serial message causes no permanent error.

Table 1 lists each job, where it runs, and whether it can block on the microcontroller.
| Job | Runs on | Blocks MCU | Notes |
| --- | --- | --- | --- |
| PWM generation for each motor | MCU | No | Hardware peripheral; set by the PID output. |
| Encoder pulse counting | MCU | No | Interrupts or hardware counter. Missed counts cause permanent drift. |
| Counts to wheel velocity | MCU | No | Input to the PID loop on every pass. |
| Acceleration ramping | MCU | No | Limits how fast each wheel's target changes, preventing current spikes on starts and reversals. |
| Per-wheel velocity PID | MCU | No | Pure arithmetic, needs a steady, guaranteed loop rate. |
| Serial communication with the Pi | MCU | Yes | Receives per-wheel velocity targets; sends cumulative tick counts and status. |
| Safety stop when commands stop arriving| MCU | No | Compares current time to the last command's arrival. |
| Stall/Jam detection | MCU | No | High PWM with near-zero speed for a minimum time (and/or high current). Cuts PWM and reports a fault. Fails safe on encoder loss. |
| Battery voltage monitoring | MCU | Yes | ADC conversion takes microseconds. The Pi has no analog inputs, so must be routed via MCU. |
| IMU Reading (not in v1) | MCU | Yes | I2C/SPI exchange with another chip. |
| Inverse kinematics | Pi | N/A | Converts robot commands to per-wheel targets. |
| Odometry integration | Pi | N/A | Computed from cumulative tick counts. |
| Receiving `cmd_vel`/publishing odometry | Pi | N/A | All ROS 2 communication stays on the PI.
| Camera capture and vision (vision not in v1) | Pi | N/A |  |
| Networking and teleop | Pi | N/A |  |
| Logging | Pi | N/A |  |
| Navigation and Autonomy (not in v1) | Pi | N/A |  |

### 2.3. Motor Control
The Pi sends per-wheel velocity commands rather than distance commands. Velocity commands are the standard
for ROS 2 mobile robots, and Nav2 and SLAM depend on smooth, continuously corrected motion. Because a velocity
command has no built-in end, the microcontroller
enforces a command timeout.

The Pi sends commands at 20 Hz, and the microcontroller stops the motors after 250 ms without a command, which
corresponds to about 25 cm of travel at 1 m/s. The velocity commands themselves act as the heartbeat: the Pi
resends the current command every cycle even
when it is unchanged, sending zeros when parked. A separate keep-alive message was rejected because a keep-alive
sender could keep running after the driving program crashes.

### 2.4. MCU-Pi Link
The two boards communicate with a custom serial protocol over a GPIO UART, without
micro-ROS; the Pi handles all ROS 2 communication. The ESP32 TX connects to the Pi RX and the
ESP32 RX to the Pi TX, with a common ground. Both boards use 3.3 V logic, so no level shifting is
needed. On the ESP32, UART2 is used (UART0 serves the USB programming and log port) on
remappable pins such as GPIO 16 and 17. On the Pi 5, the header UART is on GPIO 14 (TX) and
GPIO 15 (RX), header pins 8 and 10, and must be enabled in the Pi configuration. UART was
chosen over USB to avoid a second 5 V source feeding the ESP32.

The message format is TBD during firmware design.

### 2.5. MCU Firmware Architecture
The microcontroller is an ESP32 DevKit V1 (USB-C), already on hand and expected to use the
original ESP32 chip on an ESP32-WROOM-32 module (to be confirmed from the module
marking). It provides the MCPWM motor-control peripheral and the PCNT hardware pulse
counter for encoders. Current-sense inputs use ADC1 pins, since ADC2 is unavailable while Wi-Fi
is active, and the ADC should be calibrated.

> [!NOTE]
> Pin notes: GPIO 6 to 11 are reserved for flash; GPIO 0, 2, 12, and 15 are strapping pins and
> should be avoided; GPIO 34 to 39 are input-only (suitable for encoders and ADC, not outputs);
> ADC1 is on GPIO 32 to 39.

The firmware will use a real-time operating system, chosen primarily as a learning experience.
An RTOS is not strictly required, since only serial communication and IMU reading can block and
both have standard bare-metal solutions. The framework is leaning toward ESP-IDF, which
makes FreeRTOS the RTOS. Still open: the task and priority structure, and each task’s rate.

## 3. Motors and Wheels
### 3.1. Requirements
The target top speed is about 0.5 m/s, with slower operation during SLAM mapping. DC
gearmotors with encoders are required, because closed-loop control and odometry both
depend on feedback. To support future self-balancing, the motors need strong, fast torque, low
gear backlash, and good encoder resolution. Drive wheels are bought matched to the motor
shaft from the same supplier, so the motor is chosen first.

### 3.2. Wheels
The wheels are goBILDA 3601 Series Rhino Wheels, 72 mm diameter, with a hard 70A tread. A
hard tread keeps the wheel’s effective radius consistent, which improves odometry accuracy
compared with the soft 30A option. The wheels mount through a goBILDA hub (motor shaft to
hub to wheel), and a STEP model is available for CAD.

### 3.3. Speed Calculation
The wheel circumference is π × 72 mm ≈ 226 mm. Reaching 0.5 m/s requires 0.5 ÷ 0.226 ≈ 2.2
revolutions per second, or about 132 RPM. Allowing headroom for load, the target motor
no-load speed is roughly 165 to 190 RPM. The exact or measured circumference should be used
in odometry code, since rounding introduces cumulative error.

### 3.4. Motors
The motors are goBILDA 5203 Series Yellow Jacket planetary gear motors, 26.9:1 ratio, 223 RPM
no-load at 12 V. Key specifications: stall torque 38.0 kg·cm, stall current 9.2 A, quadrature
encoder with 751.8 PPR at the output shaft, steel planetary gears, and an 8 mm REX shaft. This
ratio was chosen because the next slower option (117 RPM) cannot reach the required 132
RPM.

The theoretical no-load top speed is about 0.84 m/s. Cruising at 0.5 m/s uses about 59% of the
no-load speed, leaving about 40% headroom, which helps with balancing recoveries and load.
Top speed can be capped in software if needed.

## 4. Motor Driver and Protections
### 4.1. Current Analysis
Motor current is given by $I=(V_(supply) - V_(backEMF))/R$, with a winding resistance of about
1.3Ω. At standstill there is no back-EMF, so each motor draws close to its 9.2 A stall current
briefly on every start from rest. A sudden full reverse at speed is the worst case: the supply
voltage and back-EMF add, briefly producing about 18 A per motor, roughly twice the stall
current.

### 4.2. Acceleration Ramp
The PID loop alone would demand full reverse output in response to a sudden command
change, producing exactly that current spike. An acceleration ramp on the microcontroller,
applied before the PID, moves each wheel’s working target toward the new command by a
limited step per control cycle. The PID therefore only ever sees a small error, and current stays
moderate during starts and reversals.

### 4.3. Stall Detection
A wheel jammed against an obstacle draws stall current continuously, and the ramp cannot help
because the target is not changing. The microcontroller therefore detects stalls: the rule trips
when PWM is high and the measured speed stays near zero for a minimum time, and it can also
use abnormally high motor current. When it trips, the microcontroller sets PWM to zero and
reports a fault to the Pi. The starting detection window is 0.5 s, to be lengthened only if false
trips occur. The rule also fails safe: if an encoder disconnects, the zero reading trips the stall rule
and stops the motors.

### 4.4. Driver Requirements
The driver requirements are: two channels; a peak rating of about 14 A per channel (1.5 times
stall current for margin) that covers about 9.2 A for at least the 0.5 s detection window; a
continuous rating of about 5 A or more per channel until normal running current is measured;
3.3 V logic compatibility; and a current-sense output, used for stall detection now and torque
control for balancing later. The driver must be a plain H-bridge so the microcontroller keeps the
control loop, which rules out smart controllers with built-in PID and R/C-PWM speed
controllers.

### 4.5. Selected Driver
The selected driver is the Pololu #2507 Dual VNH5019 Motor Driver Shield: two channels, 5.5 to
24 V operation, 12 A continuous and 30 A peak per channel, PWM up to 20 kHz, and 3.3
V-compatible inputs (2.1 V logic-high threshold). Each channel takes PWM, INA, and INB inputs,
and the control lines are broken out for use with any microcontroller. The shield format refers
only to its physical layout, so it is wired to the ESP32 with jumpers, with VDD supplied at 3.3 V
and a common ground.

Its current sense output of 0.14 V/A produces about 1.29 V at the 9.2 A stall current, which uses
much more of a 3.3 V ADC’s range than the 0.020 V/A outputs of the Pololu Dual G2 boards
(about 0.18 V). It was also the lowest-cost qualifying dual board; the additional current capacity
of the G2 boards is not needed.

## 5. Power System
### 5.1. Battery Voltage
The battery is a 3S lithium-ion pack (3.7 V nominal per cell, about 4.2 V full, about 3.0 V empty),
giving 11.1 V nominal and 12.6 V fully charged. Full charge is only about 5% above the motors’
12 V rating, and the corresponding stall current of about 9.7 A matches the driver and
stall-detection plans. Near empty, at about 9 V, the motors still reach about 167 RPM, above the
132 RPM needed for 0.5 m/s. A 4S pack was rejected because its 16.8 V full charge is about 40%
above the motor rating and raises stall current to about 12.9 A, while the extra speed is not
needed.

### 5.2. Logic Power Rail
A buck (step-down) regulator converts battery voltage to a 5.1 V rail for the Raspberry Pi 5 and
the ESP32. A linear regulator was rejected because it would waste about 21 W as heat at these
currents. Requirements: adjustable output set to 5.1 V; at least 6 A continuous, ideally about 8 A
(the Pi 5 needs up to 5 A including USB devices, plus about 0.3 A for the ESP32); and an input
range covering 9 to 12.6 V with a minimum input of about 7 V or lower, so motor-induced dips
never reach the Pi. This prevents brownouts, which can cause reboots and SD card corruption.
Motor and logic power are wired back to the battery separately, with good capacitance near the
regulator.

A buck converter cannot perform USB Power Delivery negotiation, so the Pi 5 will assume a 3 A
supply (limiting USB devices to 600 mA instead of 1.6 A) unless configured otherwise. The
configuration setting must be researched before wiring. Feeding the GPIO 5 V pins bypasses
some of the Pi’s input protection.

The ESP32 DevKit’s onboard 3.3 V regulator (3V3 pin) supplies the small 3.3 V loads: the motor
driver’s logic supply, both encoders, and later the IMU. Substantial loads must not be powered
from this pin.

### 5.3. Fuse and Wiring
A 15 A slow-blow fuse (for example, an automotive blade fuse in an inline holder) is placed at
the battery, with 14 AWG battery wiring; silicone insulation is recommended. Battery-side
currents: the logic rail draws up to about 4.2 A (5.1 V × 8 A ≈ 40.8 W output, ÷ 0.9 efficiency ≈
45.3 W input, ÷ 11.1 V); typical total draw is about 9 to 11 A; and the worst brief surge is about
25 A (both motors starting near 9.7 A each, plus logic), which a slow-blow fuse tolerates. The
fuse protects the wiring and battery from short circuits, not the electronics. A 20 A fuse was
rejected because it would require thicker wire. The rating should be rechecked once real
running current is measured; if normal draw approaches about 12 A, it must be revisited.

### 5.4. Capacity
Early iterations use a 3S pack of about 5000 mAh. At a conservative 10 A typical draw, using
about 80% of rated capacity, this gives roughly 24 minutes per charge, which is acceptable for
prototyping. The long-term goal is about one hour per session. At 10 A that would require about
12,500 mAh (about 139 Wh, roughly 0.6 to 0.9 kg), so real typical draw will be measured first; it
is likely lower than 10 A. The final choice will then be between one larger pack, two swappable
packs, or a moderate pack if the draw is low.

### 5.5. Battery Pack and BMS
Li-ion chemistry was chosen based on prior experience, over LiPo. The long-term plan is a
custom 3S Li-ion pack with a custom-designed battery management system (BMS). Whether a
commercial 3S pack is used for the first prototypes is still open.

The pack layout is 3S1P. In series, the same current flows through every cell; in parallel, current
is shared. In 3S1P each cell therefore carries the full surge of about 25 A, compared with about
12.5 A per cell in 3S2P. The pack will use high-drain 21700 cells rated comfortably above 25 A
continuous, with a capacity of about 4000 to 4500 mAh (roughly 20 minutes per charge at
about 10 A). 3S1P was chosen for its smaller size, lower weight, and simpler assembly. Because
pack current equals cell current in this layout, BMS thresholds come directly from the cell
datasheet.

The BMS sees three series groups in either layout, so capacity-dependent settings
(state-of-charge capacity, balancing time, and an optional second thermistor input) are designed
to be adjustable, keeping a future 3S2P pack possible. BMS design requirements so far: an
overcurrent threshold with a short delay that allows brief surges of about 25 A; a continuous
rating above the typical draw of about 10 A with margin; fast short-circuit protection well above
the surge level; and undervoltage cutoff near 3.0 V per cell, with the ESP32 warning earlier.

The BMS monitor IC is still open, with candidates from the TI BQ769x2 family: the BQ76972
(previously used in a university club BMS project, so schematic and configuration experience
carry over), the BQ76952, and other family members such as the BQ76942. They will be
compared on cell-count range (must cover 3S), protection and balancing features,
communication interface, package, cost, and availability, including how unused cell inputs must
be wired. A part from this family would provide programmable protection thresholds and
delays, cell balancing, high-side FET drivers with precharge, a coulomb counter, and an I2C or
SPI interface the ESP32 can read.

### 5.6. Battery Voltage Sensing
Commercial packs typically expose only their P+ and P− terminals and cut power silently when a
cell runs low. For version 1, the ESP32 therefore measures battery voltage through a resistor
divider, so it can warn, park the car, and shut the Pi down cleanly before the pack’s BMS cuts
power. A future custom BMS with a digital interface can replace the divider.
The divider uses 100 kΩ from battery positive (after the main switch, so it does not drain the
pack when off) to an ADC1 pin, and 24 kΩ from the ADC pin to ground, with a 100 nF capacitor
from the ADC pin to ground. The ratio of 0.1935 maps 12.6 V to about 2.44 V and 9 V to about
1.74 V, inside the ESP32’s recommended accurate range of about 0.15 to 2.45 V at maximum
attenuation. The divider draws about 0.1 mA (about 1.3 mW) and has a source impedance of
about 19 kΩ. Resistors should be 1% tolerance, and readings calibrated in software against a
multimeter. The ADC input itself draws negligible current, but the pin must never exceed the 3.3
V supply.

### 5.7. Main Switch
The main switch must be DC-rated for at least 15 A at more than 12.6 V; a 24 V DC rating gives
margin. The wiring order is battery, fuse, then switch. AC ratings do not apply: AC current
crosses zero many times per second, which extinguishes arcs, while a DC arc can keep burning
across opening contacts and erode or weld them. Inductive motor loads add a voltage spike
when the circuit is broken.

Early prototypes use a single DC-rated mechanical switch; a small spark at switch-on is expected
from inrush into the driver and regulator input capacitors, and anti-spark connectors are an
option. The long-term design is two-layer: a soft switch through the custom BMS’s MOSFETs for
everyday use (no arcing, soft start, and software-controlled shutdown), plus an independent
mechanical disconnect, because failed MOSFETs usually fail shorted.

## 6. System Block Diagram
Figure 1 shows the version 1 system block diagram. Thick red and black lines are battery power,
thin red and black lines are 5.1 V and 3.3 V logic power, and green lines are signals. The camera
and LiDAR connect to the Pi 5 through MIPI and USB respectively, and the laptop connects to
the Pi over Wi-Fi.
![System block diagram consisting of Power, Sensing, Computing, and Motors.]<img width="797" height="935" alt="image" src="https://github.com/user-attachments/assets/5856c310-255c-4aa1-b32a-e27e51a6c67b" />
