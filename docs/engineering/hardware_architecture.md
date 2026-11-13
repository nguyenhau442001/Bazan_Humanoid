# Hardware architecture (A+)

Electronics for P1 and later: two compute domains in the style of Renesas' robot processors, the existing hobby
servos, and position, current and temperature monitoring added around them. Parts are sold in Ho Chi Minh City. Tables between `gen:` markers are written by
`python docs/engineering/gen_docs.py` from `gen_robot.py` and `mass_report.json`.

<!-- gen:stamp -->
Generated from `design/blender/mico_parts.py`, `design/blender/mico_mass.py`, `sim/mini_humanoid/gen_robot.py`, `sim/mini_humanoid/tables/mj_servo_loads.csv`, `src/humanoid_control/scripts/mico_gait.py`, `docs/engineering/bom_aplus.csv` · source hash `eef8fd498b39`
<!-- /gen:stamp -->

## Rule

Do not upgrade hardware because a simulation result looks bad. Find the cause, measure it on the bench,
then upgrade only the part that needs it.

## Two domains

Bazan is laid out the way Renesas builds its heterogeneous robot processors (RZ/V2H: Cortex-A55 cores for Linux
and vision AI, Cortex-R8 and Cortex-M33 cores for real-time work). There are two domains with a hard rule between
them: **the application domain plans, the real-time domain acts.** Renesas puts both domains on one chip; Bazan
puts them on two boards joined by a serial link. Everything below follows from that one difference.

```
┌──── APPLICATION DOMAIN · Cortex-A55 × 4 (Radxa Zero 3W, RK3566) ─────────────────────────────────────────────┐
│ Linux · ROS 2 Jazzy · camera + NPU (vision, gestures) · ToF · OLED face · microphones · path planning · gait │
│ Plans motion as joint trajectories. Allowed to be late. Never closes a servo loop.                           │
└───────────────────────────────────────────────┬──────────────────────────────────────────────────────────────┘
               Traj (≥ 100 ms ahead) / Hello  ↓ │ ↑  State 100 Hz · Diag 1 Hz · Info · Log       UART 921600
┌───────────────────────────────────────────────┴─── REAL-TIME DOMAIN · Cortex-M4F (STM32F411 Black Pill) ─────┐
│ FreeRTOS. 200 Hz control task: sense · safety · interpolate · PWM. Supervisor task owns OE and the IWDG      │
│   ├─ I²C1 ── PCA9685 #1 ── PWM, 12 leg servos         ├─ ADC ── CD74HC4067 ── 12 leg pots + 4 FSR            │
│   ├─ I²C1 ── PCA9685 #2 ── PWM, arms and neck         ├─ ADC ── NTC × 2 ── hip-roll servo cases              │
│   └─ I²C2 ── INA226 × 2 (leg branches) · MPU6050                                                             │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
2S LiPo → fuse → 6 V UBEC (servos)  ·  5 V buck (Radxa)  ·  3.3 V LDO (logic)
```

### Application domain: Cortex-A55 × 4 (Radxa Zero 3W)

| Owns | Does not own |
| --- | --- |
| ROS 2 graph, launch, logging | Any timing below about 50 ms |
| Camera and NPU inference: person, hand and gesture detection | Servo pulses, servo power |
| ToF, microphones, OLED expressions | The decision to stop: it can only request one |
| Path planning, footstep planning, gait and trajectory generation | |
| The ros2_control hardware interface that talks to the MCU | |

Linux can pause for tens of milliseconds (I/O, thermal throttling, a busy NPU), so this domain is treated as
untrusted for timing. It sends trajectories, not setpoints: every message carries at least 100 ms of motion, so a
late packet changes nothing the legs do.

### Real-time domain: Cortex-M4F (STM32F411)

| Owns | Does not own |
| --- | --- |
| The 200 Hz loop: trajectory interpolation, sensing, safety checks | Planning: it never decides where to step |
| Servo PWM (written at the 50 Hz servo frame) and both OE lines | Vision, audio, anything with Linux drivers |
| Pots, FSR, rail current and voltage, case temperature, IMU | |
| Modes and safe stop: HOLD, CROUCH, FAULT; the deadline supervisor and the ~50 ms IWDG | |

This is the only domain that can power the servos, and every stop decision is made here without asking the
application domain. Firmware: `firmware/stm32f411` (FreeRTOS, CubeMX layout); the hardware-independent logic is
`lib/core`. Task model below.

### Real-time firmware (M0)

| Task | Priority | Period | Owns |
| --- | --- | --- | --- |
| Supervisor | 5 (top) | 1 ms | Both OE pins and the IWDG refresh. Checks every control cycle against its deadline |
| Control | 4 | 5 ms (200 Hz) | Sensors → `Controller` (modes, safety, trajectory) → PWM. Never waits on the link |
| Comm | 3 | 1 ms | USART1 RX DMA (circular) → control; control and Diag frames → TX DMA |
| Diagnostics | 1 | 1 s | `Diag`: timing (max and p99), CPU per task, stack headroom, heap, faults, reset cause |

- Tasks share no peripheral. They exchange data only through lock-free single-producer rings, a seqlock
  snapshot and the supervisor's atomics; no mutex sits on the control path.
- All task memory is static. The FreeRTOS heap (24 KB, heap_4) is kept for micro-ROS in M1; newlib's `_sbrk`
  refuses, and every allocation after the scheduler starts is counted in `Diag.allocs_after_start`.
- Clocks: TIM5 is a 32-bit 1 MHz monotonic counter (control timing and the run-time stats); SysTick is the
  FreeRTOS tick; TIM11 is the HAL timebase.
- The supervisor accepts only valid cycles: finished within 5 ms of their own release, releases on the 5 ms grid.
  It latches `CONTROL_DEADLINE` on an invalid cycle or when no cycle has completed by the next deadline. A latch
  cuts OE at once (within 1 ms). A stall, or 20 invalid cycles in a row, also stops the IWDG refresh, so the MCU
  resets 34–94 ms later (LSI tolerance); a single late cycle does not reset it.
- OE is on only when the controller asks, the supervisor sees control on time, and nothing is latched. After
  any reset both OE lines are high (pull-ups, then GPIO init before any other peripheral) and a watchdog reset
  boots into FAULT `WATCHDOG_RESET`: nothing re-enables the servos without the host's ClearFault and Enable.
- `ClearFault` is refused while its cause is still present (supervisor unhealthy, over-current, NTC hot or
  open, I²C failing); the refusal is logged with the reason.
- M0 results and the hardware acceptance procedure: [m0_report.md](m0_report.md).

### The boundary

| Crosses | Direction | Rate | Size |
| --- | --- | --- | --- |
| `Traj`: up to 8 points, each `dt_ms` + 20 angles | A → M | as needed, ≥ 100 ms buffered | 2 + 42 × n bytes |
| `Hello`: heartbeat while standing still | A → M | 20 Hz when no trajectory is queued (gap < 100 ms) | 0 bytes |
| `Enable`, `ClearFault`, `RawPulse` | A → M | on request | ≤ 3 bytes |
| `TestHook`: overrun or hang injection, M0 acceptance image only | A → M | on request | 3 bytes |
| `State`: angles, pots, FSR, current, voltage, temperature, IMU, mode, faults | M → A | 100 Hz | 131 bytes |
| `Diag`: loop timing, CPU, stacks, heap, supervisor, reset cause, fault history | M → A | 1 Hz | 78 bytes |
| `Info`, `Log` | M → A | on request / on events | small |

Budget: a `State` frame is about 140 bytes on the wire, 1.5 ms at 921600 baud, so the link runs at about 15% load.
When a side fails:

- Application domain silent (crash, cable, reboot): the MCU finishes the queued motion, holds 1 s, then crouches
  slowly with the servos powered.
- Real-time domain late: the supervisor latches `CONTROL_DEADLINE` and cuts OE within 1 ms of the missed
  deadline; the MCU keeps running and reports the fault.
- Real-time domain hung: the supervisor stops refreshing the IWDG, which resets the MCU in 34–94 ms; OE pull-ups
  keep the servos off through the reset, and `State` reports `WATCHDOG_RESET` until cleared.
- Electrical fault (over-current, over-temperature, under-voltage, jammed joint): the MCU cuts the outputs and
  latches FAULT. Only the application domain's `ClearFault` releases it, after the cause is gone.

## Compared with Renesas

Reference: Renesas proposes the RZ/V2H for vision AI and gesture response in humanoids (4 × Cortex-A55, 2 ×
Cortex-R8, 1 × Cortex-M33, DRP-AI3 accelerator), RZ/T and RZ/N parts for multi-axis motion control, and RA
Cortex-M MCUs for joint-level motor control with micro-ROS over CAN FD.

| Aspect | Renesas | Bazan | Same? | What it means for Bazan |
| --- | --- | --- | --- | --- |
| Application cores | Cortex-A55 × 4 (RZ/V2H) | Cortex-A55 × 4 (RK3566) | Same | Same instruction set and Linux / ROS 2 stack; code ports either way |
| AI accelerator | DRP-AI3, 10 TOPS/W, up to 80 TOPS with pruning | RK3566 NPU, about 1 TOPS class | Much smaller | Small models only: hand landmarks and gesture classes at low resolution. Measure fps before promising a demo |
| Real-time cores | Cortex-R8 × 2 and M33, on the same die | Cortex-M4F on a separate board | Different | Same split of jobs, but the domains talk over a UART instead of on-chip memory |
| Link between domains | On-chip messaging between cores | 921600-baud serial, COBS + CRC frames | Different | Microseconds become milliseconds: hence trajectories with ≥ 100 ms ahead, and every safety decision on the MCU |
| Joint control | One RA MCU per joint, closed-loop motor control, micro-ROS over CAN FD | Hobby servos with their own analog loop; the MCU only sends pulses and reads pots | Different | No torque or current control per joint, so no compliance or impedance control. A+ gives position, current per leg and temperature; an upgrade comes only if P0 shows the need |
| Middleware to the joints | micro-ROS | Custom binary protocol now, micro-ROS from M1 | Same from M1 | Decided, below |
| Safety | Functional-safety features in the RZ/T and RZ/N lines | Deadline supervisor, IWDG, latched faults, one owner for OE | Weaker | Fine for a desk robot on a harness; not a certified design |
| Cost | Evaluation kits | Radxa + Black Pill, parts from HCMC | | The point of Bazan: the same architecture at hobby cost |

## Protocol between the domains: micro-ROS (decided)

micro-ROS over Micro XRCE-DDS on the UART is the official link from M1. The custom protocol stays for M0 bring-up,
the P0 bench and the acceptance tools. Conditions attached to the decision: compact fixed-size embedded messages
(sizes checked against the real CDR serialisation), a monotonic hardware clock on the MCU with time sync,
trajectories rejected when sync is lost or the offset is too large, a benchmarked 40 ms latency budget, trajectory
expiry with a safe-state policy, a single owner for OE, and a `clear_fault` that cannot bypass a present
condition. The micro-ROS commit SHA and agent image digest are pinned and verified for Jazzy in M1.

| | Custom protocol (M0, bench) | micro-ROS on the MCU (M1 on) |
| --- | --- | --- |
| Status | Written and tested: 361 regression checks, Python and C++ agree | M1 |
| Renesas likeness | Lower | Matches the RA joint-MCU pattern |
| ROS 2 side | A ros2_control hardware interface decodes the frames | MCU publishes and subscribes ROS 2 topics through the micro-ROS agent |
| Cost on the F411 | Small, fixed buffers | More RAM and flash (fits in 128 KB RAM), more moving parts |
| Timing | Fixed frame sizes, easy to budget | Best-effort topics needed to keep latency predictable |

Either way, `lib/core/controller` (modes, trajectory, safety) stays the same; only the transport changes.

An MCU from Renesas is also possible: the Arduino Nano R4 (RA4M1, Cortex-M4F 48 MHz, 32 KB RAM, 45 × 18 mm). It is
weaker than the F411 (48 vs 100 MHz, 32 vs 128 KB RAM), tight for micro-ROS, and not yet found in HCMC shops, so
the F411 stays unless a Renesas part becomes a hard requirement.

## Actuators

<!-- gen:servos -->
| Servo | Count | Stall kg·cm | g each | g total | Joints |
| --- | ---: | ---: | ---: | ---: | --- |
| MG90S | 4 | 2.20 | 13.4 | 53.6 | hip_yaw, shoulder_pitch |
| MG92B | 10 | 3.50 | 14 | 140.0 | ankle_pitch, ankle_roll, hip_pitch, hip_roll, knee |
| SG90 | 6 | 1.80 | 9 | 54.0 | elbow, head_pitch, neck_yaw, shoulder_roll |
| **Total** | 20 |  |  | **247.6** | 37% of 677 g |
<!-- /gen:servos -->

All MG92B stay. No smart servos for now: the only small one with enough torque (Feetech STS3032, 4.5 kg·cm,
20 g) is import-only, about 1M VND each, and adds 72 g over the twelve leg joints. See `torque_validation.md`
for why the current servos are not ruled out.

## A+ monitoring, with its conditions

**Servo position: external joint-position monitoring.** A wire on the wiper of each leg servo's internal pot.
This is monitoring, not smart-servo feedback: the servo still closes its own loop. It gives the real angle, the
error to the command, and stall or jam detection. It does not give torque or velocity control.

- Measure first, on one opened servo, before anything is wired to the MCU: wiper voltage range over the full
  travel, and whether the servo's own position changes when a probe is attached.
- Buffer every wiper with an op-amp follower so the ADC never loads the servo's feedback. LMV358 (two per
  chip, powered from 5 V) takes inputs up to about 4 V; if P0 shows wipers higher, use MCP6002 (rail-to-rail
  input) or a high-value divider ahead of the buffer. Divide the buffer output down to 3.3 V if needed, and
  put a series resistor plus clamp at each ADC pin.

**CD74HC4067 mux.** The Blackpill's STM32F411CEU6 has 10 ADC inputs; 12 pots and 4 FSR need 16.
16 channels at 200 Hz is 3200 samples/s, light for the F411.

- After each channel switch, wait for the input to settle; discard the first conversion if needed.
- Low-pass filter each channel (moving average or first-order IIR).
- Calibrate every pot (two known angles per joint) and every FSR on its own foot. No shared thresholds.

**PCA9685 for the leg PWM.**

- It generates signals only. Servo power comes from the 6 V rail, never through the board's V+ trace.
- OE goes to an STM32 pin, pulled high (outputs off) at power-up until the firmware is ready.
- Safe stop needs care on a biped: outputs off means the servos go limp and the robot drops. So:
  - Lost commands from the Radxa (no trajectory for 100 ms): hold the last pose, then crouch slowly.
  - Electrical faults (over-current, NTC over-temperature, rail under-voltage): OE off. A fall is cheaper
    than a burnt servo. Run P1 on a harness until this is proven.
  - STM32 independent watchdog (IWDG): a hung MCU resets, and OE stays high until it is back.

**INA226 × 2.** One per leg branch (6 servos each), with rail voltage. The shunt must suit the branch:
INA226 reads ±81.92 mV full scale, so a 0.1 Ω shunt saturates at 0.8 A. 10 mΩ gives 8.2 A full scale,
2.5 µV LSB and 0.25 W at 5 A. The CJMCU-226 module sold in HCMC already carries a 10 mΩ shunt: check that
the resistor reads **R010**, not R100, before buying, and ignore the 3.2 A rating on the shop page (it is
copied from INA219 listings). The 2512 shunt is rated 1 W; mount the module away from the servo bundle.

**NTC × 2.** 10 kΩ NTC (MF52) glued to each hip-roll servo case with thermal epoxy. Shops do not state the
B value, so calibrate each one against a thermometer at two temperatures. Case temperature lags the motor
winding, so set the limits conservatively and treat them as a trend, not a safe value.

## Power

- 2S LiPo 1100 mAh 30C (in the BOM), fuse, reverse protection. Capacity is re-checked against P0 current data.
- Servo rail 6.0 V from the 6 V 8 A UBEC in the BOM. Upper bound: ten MG92B stalled at once is about 8.5 A,
  using a reported 0.85 A stall current that P0 must confirm. If P0 shows peaks above 8 A, the UBEC is the
  part to upgrade. Bulk capacitors at each leg.
- Compute rail 5 V from the 5 V 3 A mini buck in the BOM (Radxa recommends at least 2 A).
- Logic 3.3 V from an LDO. All grounds join at one point near the battery.

Three load states to size against: idle (standing, display and sensors on), walking (12 leg servos
moving), peak (turning, stepping onto one foot, recovering).

## Parts to add for A+

Already in the BOM (`docs/design/sketch_shell_a`, ≈ 5.65M VND): STM32F411 Black Pill, PCA9685 × 2, MPU6050,
FSR × 4, 2S LiPo, 6 V 8 A UBEC, 5 V buck. A+ adds only the monitoring parts below. Prices are list prices from
HCMC shops, checked on the date in `bom_aplus.csv`; "listed" means the shop page showed a price, not that stock
was confirmed. Call the shop before going.

<!-- gen:bom -->
| Part | Qty | VND each | VND | Shop | Note |
| --- | ---: | ---: | ---: | --- | --- |
| CD74HC4067 16-channel analog mux module | 1 | 15,000 | 15,000 | [icdayroi](https://icdayroi.com/module-giao-tiep-ky-thuat-so-cd74hc4067-cmos-16-kenh) | 12 servo pots + 4 FSR into one ADC pin |
| INA226 current/voltage module CJMCU-226 (R010 shunt) | 2 | 65,000 | 130,000 | [icdayroi](https://icdayroi.com/cam-bien-dong-cjmcu-226-ina226-i2c) | same price at Điện tử Tuyết Nga; check the shunt reads R010 |
| NTC 10 kΩ thermistor MF52-103 | 4 | 1,500 | 6,000 | [icdayroi](https://icdayroi.com/dien-tro-nhiet-ntc-mf52-103-dien-tro-nhiet-10k) | 2 for the hip-roll servos + 2 spares |
| LMV358 dual op-amp SOIC-8 | 6 | 7,000 | 42,000 | [icdayroi](https://icdayroi.com/lmv358idr) | 12 wiper buffers; MCP6002 (rail-to-rail input, Thế Giới IC) if P0 shows wipers near the supply |
| Perfboard, headers, series resistors, clamp diodes | 1 | 40,000 (est.) | 40,000 | Nhật Tảo | monitoring board for P0 and P1 |
| **A+ total** |  |  | **233,000** |  |  |
| **Robot total** (BOM + A+) |  |  | **5,883,000** |  |  |

Prices checked 2026-11-07.
<!-- /gen:bom -->

## Prototype plan

| Stage | Build | Done when |
| --- | --- | --- |
| P0 | Actuator bench: MG92B, MG90S, SG90, plus the A+ monitoring chain | Torque, current, temperature and speed data for each servo type |
| P1 | Pelvis + 12 DoF legs + feet with FSR + IMU, on a harness | Stands, shifts weight, lifts one foot, steps in place, walks, turns |
| P2 | Torso, arms, neck, OLED face, camera, ToF | Gestures and expressions while standing |
| P3 | Full Bazan: 20 DoF, battery, ROS 2 on the Radxa | The MuJoCo demo on the floor: walk, turn 60°, wave |

### P0: MG92B hip-roll thermal test

Run on two servos (one per side).

| Item | Value |
| --- | --- |
| Supply | 6.0 V, current-limited bench supply |
| Continuous load | 1.07 kg·cm |
| Transient load | 2.2 kg·cm for 0.4 s in every 2 s cycle |
| Duration | 10 min, or until a stop condition |
| Stop if | case above 60 °C, or current above 90% of the measured stall current for more than 1 s |

Load without a load cell: a lever on the horn with a known mass. At 20 mm from the axis, 53.5 g gives
1.07 kg·cm; a second mass that the lever lifts during the transient adds the rest. Check the arm length with
calipers; that is the calibration.

Log at 10 Hz or faster: case and ambient temperature, current, rail voltage, commanded and measured angle
(from the pot), position error, and time above each temperature threshold.

Pass: position error stays within 3° and the case stays below 60 °C for the full 10 minutes on both servos.
