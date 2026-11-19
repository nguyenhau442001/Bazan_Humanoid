# Bazan mainboard (BZN-MB) rev A: requirements

Step 3 of the mainboard work. Every later step (block diagram, part choice, schematic, layout, review, bring-up)
is checked against this page. A requirement that changes is changed here first.

Rev A is a **carrier board**: the STM32F411 Black Pill stays a socketed module, the 6 V UBEC and the 5 V buck for
the Radxa stay modules. Rev A replaces the perfboard, both PCA9685 modules, both INA226 modules, the MPU6050 module and
their sleds and harness. Chip-down MCU is rev B, after rev A has run on the robot.

Sources: firmware pin map `software/firmware/stm32f411/Core/Inc/main.h`, channel tables
`software/core/generated/joint_table.h`, architecture and power notes
`docs/engineering/hardware_architecture.md`, module envelopes `hardware/mechanical/electronics.py`.

## 1. Functions

| ID | Requirement |
| --- | --- |
| F-1 | Drive 20 servos: 12 leg servos from PCA9685 #1 (I²C1, 0x40, channels 0–11 in `JOINT_NAMES` order), 6 arm and 2 neck servos from PCA9685 #2 (I²C1, 0x41, channels 0–7). 50 Hz frames, 500–2500 µs. |
| F-2 | Each PCA9685 has its own OE line to the MCU (PB0 legs, PB1 arms), pulled high on the board so outputs are off until the firmware drives them. |
| F-3 | Measure current and bus voltage of every servo branch with an INA226 on I²C2: left leg 0x44, right leg 0x45, arms + neck 0x46. |
| F-4 | Read 12 leg-servo pot wipers and 4 foot FSRs through one 16-channel analog mux (CD74HC4067, S0–S3 on PB12–PB15, SIG on PA0 / ADC1_IN0). Each wiper is buffered by an op-amp follower before the mux. |
| F-5 | Read 2 NTC 10 kΩ thermistors on the hip-roll servo cases (PA1 left, PA2 right). |
| F-6 | Link to the Radxa: USART1 PA9 TX, PA10 RX, 921 600 baud, 3.3 V logic, plus ground. |
| F-7 | MPU6050 (6-axis, no magnetometer) on the board, I²C2 0x68, rigid to the torso, axes printed on the silkscreen and aligned with `base_link`. Its position becomes `imu_link` in the CAD. |
| F-8 | SWD header (SWDIO PA13, SWCLK PA14, NRST, 3.3 V, GND) reachable with the board in the torso. |
| F-9 | Hardware over-current stop: either INA226 ALERT forces both OE lines high without the MCU. The MCU can read that the trip happened. |

Every pin, address and channel above is the one the firmware already uses, except two additions that need a few
firmware lines: the third INA226 (0x46) and the F-9 trip input (a free pin, chosen in step 6). The board works
with today's firmware; the additions are simply not read until then.

## 2. Power

| ID | Requirement |
| --- | --- |
| P-1 | Servo input 6.0 V from the UBEC (6 V 8 A cont., 15 A burst). Board path rated 8.5 A continuous, the estimate for ten MG92B stalled at once (0.85 A each, to be confirmed in P0), 15 A for 1 s. |
| P-2 | Servo power splits on the board into three branches, each through its own fuse and INA226 shunt: left leg (6 servos), right leg (6), arms + neck (8). |
| P-3 | Shunts 10 mΩ, 2512, ≥ 1 W, Kelvin-connected. INA226 full scale is ±81.92 mV, so 8.2 A per branch. |
| P-8 | Nothing on the board may overheat, burn or vent with any one fault: a shorted servo, a shorted branch, a stalled leg, battery reversed, or a connector plugged one pin off. The fuse or the hardware trip clears each before any part passes its rating; step 5 shows the numbers. |
| P-4 | Reverse-polarity protection and a fuse on the 6 V input; a bulk capacitor at each branch (value set in step 5). |
| P-5 | Logic 3.3 V comes from the Black Pill's own regulator or an on-board LDO (decided in step 4). Analog 5 V for the op-amps is low-noise and separate from the servo rail. |
| P-6 | The board never carries the Radxa's 5 V current; the buck feeds the Radxa directly. Grounds join at one point. |
| P-7 | Measure battery voltage (2S LiPo, 6.0–8.4 V) for the under-voltage rule, on the INA226 bus input or a divider. |

## 3. Signal integrity and measurement

| ID | Requirement |
| --- | --- |
| S-1 | Pot reading error from ground shift (servo current in the servo ground wire) is measured in P0 and kept below 1° at the joint. The board provides a ground-sense input per leg servo if P0 shows it is needed. |
| S-2 | Every ADC input has a series resistor and clamp; the mux SIG line settles within the 5 µs the firmware waits (`read_mux`). |
| S-3 | Analog section (op-amps, mux, ADC lines, NTC dividers) on its own ground region, away from the servo current path. |
| S-4 | Each PWM output has a series resistor (about 220 Ω) so an unpowered board is not back-fed by a powered servo. |
| S-5 | I²C pull-ups sized for the bus: both buses stay on the board. |

## 4. Connectors

| ID | Requirement |
| --- | --- |
| C-1 | 20 servo headers, standard 2.54 mm 3-pin (GND, V+, signal), polarity printed on the silkscreen, grouped legs / arms. |
| C-2 | Power input with a locking connector rated ≥ 15 A (XT30 class). |
| C-3 | Sensor connectors keyed and locking (JST-XH or GH): 12 wipers (+ ground sense if S-1 needs it), 4 FSR, 2 NTC, Radxa UART. |
| C-4 | No connector a servo plug fits into except the servo headers. |

## 5. Mechanical

| ID | Requirement |
| --- | --- |
| M-1 | Outline at most 80 × 60 mm, four M2 mounting holes (Ø2.2). A new sled is printed for it (nothing is printed yet). |
| M-2 | Height above the board with the Black Pill socketed at most 15 mm (the perfboard stack was 21 mm). |
| M-3 | All connectors on edges reachable from the torso opening; SWD and the Black Pill USB-C reachable without taking the board out. |
| M-4 | Board outline, holes and tallest parts go into `hardware/mechanical/electronics.py` so the CAD clearance checks cover it. |

## 6. Manufacture

| ID | Requirement |
| --- | --- |
| D-1 | JLCPCB standard process: 4 layers, 1.6 mm, 1 oz outer copper (2 oz outer if the 8.5 A path needs it), 0.15 mm track and space or larger. |
| D-2 | SMD parts on the top side only, from LCSC stock (basic parts preferred), so JLCPCB can assemble them. Through-hole headers and sockets hand-soldered. |
| D-3 | Every net that bring-up needs has a test point: 6 V in, each branch after its shunt, 5 V analog, 3.3 V, GND, I²C1, I²C2, mux SIG, OE lines. |
| D-4 | Sources (schematic as code, layout), generated outputs (Gerber, drill, BOM, pick-and-place) and check reports live in this folder. |

## 7. Open items that need P0 data

1. Servo stall current (P-1) and the real peak during a step.
2. Pot wiper voltage range over the full travel: decides the op-amp (LMV358 or MCP6002) and the divider.
3. Ground-shift error on the wiper with the motor loaded (S-1).
