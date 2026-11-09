# Mico MCU firmware (STM32F411, FreeRTOS)

Real-time domain of [`docs/engineering/hardware_architecture.md`](../../docs/engineering/hardware_architecture.md):
STM32F411CEU6 Black Pill, FreeRTOS V11.1.0, ST HAL. Servo PWM through two PCA9685, pot, FSR, current, voltage and
temperature monitoring, the safe-stop rules, a deadline supervisor and the hardware watchdog. M0 status and the
board acceptance procedure: [`docs/engineering/m0_report.md`](../../docs/engineering/m0_report.md).

| Path | What |
| --- | --- |
| `../../lib/core/` | All decisions, no hardware: controller (modes, safety), trajectory, supervisor, loop statistics, link framing, lock-free ring and seqlock, generated tables. Host-tested |
| `Core/` | CubeMX layout: `main.c` (clock, GPIO with OE high first, DMA, USART1, I²C1/2, ADC1, TIM5, IWDG), MSP, interrupts, FreeRTOS config, start-up, newlib stubs |
| `app/` | The four FreeRTOS tasks (`tasks.cpp`), `Diag` assembly, `board.h` (the hardware interface each task may use) |
| `hardware/board_stm32.cpp` | `board.h` on the HAL: PCA9685, INA226, MPU6050, mux ADC, NTC, OE, IWDG, UART DMA |
| `Drivers/`, `FreeRTOS/` | ST HAL, CMSIS, FreeRTOS kernel: vendored at pinned commits by `tools/fetch_drivers.py` (see `Drivers/VERSIONS.md`) |
| `test/posix/` | The same tasks on the FreeRTOS POSIX port with a simulated board: overrun, hang, watchdog-reset and OE scenarios |
| `config/calibration.json` | Per-joint servo and pot calibration, NTC, FSR |
| `tools/` | `gen_tables.py` (generated headers), `fetch_drivers.py`, `mico_link.py` (host tool, M0 acceptance) |

## Tasks

| Task | Priority | Period | Owns |
| --- | --- | --- | --- |
| supervisor | 5 | 1 ms | OE pins, IWDG refresh, deadline checks of the control task |
| control | 4 | 5 ms | sensors → `Controller` → PWM (legs on cycle 0 mod 4, arms on 2 mod 4); never waits on the link |
| comm | 3 | 1 ms | USART1 RX DMA ring → control; control and Diag frames → TX DMA |
| diagnostics | 1 | 1 s | `Diag`: exec and jitter (max, p99), missed deadlines, CPU per task, stack headroom, heap, faults |

Tasks share no peripheral and no lock. Task memory is static; the 24 KB FreeRTOS heap is kept for micro-ROS (M1).

## Build

Needs the Arm GNU toolchain 13.x, CMake ≥ 3.22 and Ninja. On Windows (PowerShell):

```powershell
winget install Arm.GnuArmEmbeddedToolchain Kitware.CMake Ninja-build.Ninja
cmake -S firmware\stm32f411 -B firmware\stm32f411\build -G Ninja `
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build firmware\stm32f411\build
```

Images: `mico_mcu.elf/.hex/.bin` (release) and `mico_mcu_m0test.*` (adds `TestHook` overrun and hang injection,
for the acceptance run only). Add `-DMICO_BENCH=ON` for the P0 bench (one PCA9685, no IMU).

STM32CubeIDE: *File → Import → Existing CMake project* on `firmware/stm32f411`, or open it with the CMake support of
VS Code + STM32 extension. There is no `.ioc` yet (see the M0 report); peripheral settings are in `Core/Src/main.c`
and `Core/Src/stm32f4xx_hal_msp.c` in CubeMX's layout.

Flash with an ST-LINK (STM32CubeProgrammer):

```powershell
STM32_Programmer_CLI -c port=SWD -w firmware\stm32f411\build\mico_mcu.hex -v -rst
```

or over USB with the ROM bootloader: hold **BOOT0**, tap **NRST**, release BOOT0, then
`STM32_Programmer_CLI -c port=USB1 -w firmware\stm32f411\build\mico_mcu.bin 0x08000000 -v -go`.

Host tests (WSL or Linux; CI runs them on every push):

```bash
make -C lib/core test                      # 361 regression + 4129 safety checks (ASan/UBSan), threads (TSan)
make -C firmware/stm32f411/test/posix test # FreeRTOS tasks on the POSIX port
make -C firmware/stm32f411/test/posix accept-sim   # m0-accept against the simulated MCU (pty)
python firmware/stm32f411/tools/gen_tables.py --check
python firmware/stm32f411/tools/fetch_drivers.py --check
```

After any change to `gen_robot.py`, `mico_gait.py` or `config/calibration.json`: `python tools/gen_tables.py`, then
rebuild. CI fails if the generated files are stale.

## Wiring

| Signal | Pin | Notes |
| --- | --- | --- |
| I²C1 SDA / SCL | PB7 / PB6 | 400 kHz. PCA9685 #1 legs (0x40), PCA9685 #2 arms + neck (0x41: solder A0) |
| I²C2 SDA / SCL | PB3 / PB10 | 400 kHz. INA226 left 0x44 (A1 to VS), right 0x45 (A1, A0 to VS), MPU6050 0x68 |
| OE legs / arms | PB0 / PB1 | To each PCA9685 OE, **10 kΩ pull-up to 3.3 V**: outputs stay off through reset |
| Mux S0..S3 | PB12..PB15 | CD74HC4067, EN to GND |
| Mux SIG | PA0 | Channels 0..11 leg pots (wire order below), 12..15 FSR l_heel, l_toe, r_heel, r_toe |
| NTC left / right | PA1 / PA2 | 10 kΩ from 3.3 V to the pin, NTC from the pin to GND, 100 nF to GND |
| Host link | PA9 TX / PA10 RX | USART1, 921600 8N1, 3.3 V. To the Radxa, or to a PC through a 3.3 V USB-UART adapter (CP2102 or CH340) |
| Status LED | PC13 | on: IDLE · off: running · blinking: FAULT |

USB-UART adapter: measure that its TX idles at 3.3 V (many CH340/CP2102 boards default to 5 V), share GND, cross
TX/RX (adapter TX → PA10, adapter RX ← PA9), and do not feed the adapter's 5 V into a Black Pill that is also on USB.

The FreeRTOS firmware talks on USART1 only (USB CDC was dropped: micro-ROS runs on this UART from M1). PCA9685 V+
is **not** the servo supply path. Each pot wiper goes through its op-amp buffer before the mux.

Joint order on the wire and on the PCA9685 channels is `mico_gait.JOINTS`: legs board channels 0..11 = l_hip_yaw,
l_hip_roll, l_hip_pitch, l_knee, l_ankle_pitch, l_ankle_roll, then the same for the right leg; arms board channels
0..7 = l_shoulder_pitch, l_shoulder_roll, l_elbow, r_shoulder_pitch, r_shoulder_roll, r_elbow, neck_yaw, head_pitch.

## Modes and safe stop

| Mode | Servos | Enters when | Leaves when |
| --- | --- | --- | --- |
| IDLE | limp (OE off) | boot, `Enable(0)`, `ClearFault`, calibration timeout | `Enable(1)` starts at the measured pose, or the stand pose if pots are not calibrated |
| RUN | powered | `Enable(1)`, any trajectory | trajectory finished **and** nothing heard for 100 ms: HOLD |
| HOLD | powered, last pose | lost host | 1 s later: CROUCH; a new trajectory: RUN |
| CROUCH | powered | 1 s in HOLD | slow 2 s move to the crouch, then stays; a new trajectory: RUN |
| FAULT | limp (OE off), latched | over-current, over-temperature, under-voltage, joint jam, I²C failure, watchdog reset, control deadline | `ClearFault`, only when the cause is gone |
| CALIB | one servo | `RawPulse` from IDLE | 500 ms without a message: IDLE |

OE is on only while the controller asks for it **and** the supervisor sees every control cycle finish on time
**and** no deadline fault is latched. A late cycle latches `CONTROL_DEADLINE` and cuts OE within 1 ms; a stalled
control task also stops the IWDG refresh, so the MCU resets in 34–94 ms and boots in FAULT `WATCHDOG_RESET`.
Nothing re-enables the servos after a reset. Thresholds are in `lib/core/config.h`.

## Protocol

Frames are COBS(type, seq, payload, CRC-16/CCITT-FALSE little-endian) followed by `0x00`. Little-endian payloads,
layouts in `lib/core/generated/protocol.h` and `tools/mico_protocol.py` (generated together).

| Type | Direction | Payload |
| --- | --- | --- |
| `0x01 Hello` | host → MCU | none; answered by `Info` |
| `0x02 Enable` | host → MCU | `on` |
| `0x03 Traj` | host → MCU | `flags` (bit 0: replace queue), `n` ≤ 8, then n × (`dt_ms`, 20 × int16 angle in 1e-4 rad) |
| `0x04 ClearFault` | host → MCU | none; refused (and logged) while a cause is present |
| `0x06 RawPulse` | host → MCU | `joint`, `pulse_us` (calibration) |
| `0x07 TestHook` | host → MCU | `action` (1 overrun, 2 hang), `arg_ms`; only in `mico_mcu_m0test` |
| `0x80 State` | MCU → host | 100 Hz: time, mode, faults, angles, raw mux ADC, temperatures, currents, rail voltages, IMU, worst loop time, link errors |
| `0x81 Info` | MCU → host | firmware and protocol version, joint count, joint-table hash |
| `0x82 Log` | MCU → host | level, text |
| `0x83 Diag` | MCU → host | 1 Hz: cycles, missed deadlines, exec avg/p99/max, jitter p99/max, supervisor wake jitter, longest IWDG refresh gap, last latch delay, CPU per task, stack headroom, heap, allocations after start, link drops, supervisor and OE bits, reset cause, last 4 fault changes |

## Host tool

```powershell
pip install pyserial
python firmware\stm32f411\tools\mico_link.py --port COM5 info
python firmware\stm32f411\tools\mico_link.py --port COM5 diag
python firmware\stm32f411\tools\mico_link.py --port COM5 monitor --csv run.csv
python firmware\stm32f411\tools\mico_link.py --port COM5 m0-accept --minutes 10 --report m0_accept.json
```

`calibrate` and `p0-thermal` work as before (see the hardware doc for the P0 rig):

```powershell
python firmware\stm32f411\tools\mico_link.py --port COM5 calibrate --joint l_knee
python firmware\stm32f411\tools\mico_link.py --port COM5 p0-thermal --joint l_hip_roll --hold-deg 0 --lift-deg 10 --stall-a 0.85 --csv p0_left.csv
```

Calibration: the tool pulses one servo at 1100 µs and 1900 µs; type the measured joint angle at each (URDF sign),
paste the printed line into `config/calibration.json`, run `tools/gen_tables.py`, rebuild and flash.
