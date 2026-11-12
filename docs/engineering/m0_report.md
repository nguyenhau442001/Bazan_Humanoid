# M0 report: STM32F411 FreeRTOS foundation

M0 moves the real-time domain from the Arduino loop to FreeRTOS. Scope: the CubeMX-layout project, FreeRTOS with
a 200 Hz control task and a top-priority supervisor, the UART DMA foundation and a diagnostics task.
`lib/core` was ported, not rewritten. There are no micro-ROS entities and no actuator work in M0.

| | Status |
| --- | --- |
| Host verification | **PASS** |
| Board acceptance | **PENDING**: one `m0-accept` run on the STM32F411 (procedure below) |

Host results never stand in for board timing. M0 closes, and M1 starts, only after the board report is reviewed.

## Acceptance criteria

Each line says where it is proven: **host** (unit tests, POSIX integration, CI) or **board**.

| Criterion | Evidence | Host | Board |
| --- | --- | --- | --- |
| 357 regression checks pass | `make -C lib/core test` (ASan + UBSan), CI | **Pass**: 361 (the 357, one scenario adapted to the stricter ClearFault rule, plus 4) | – |
| 200 Hz | Cycle count against MCU time and against host time | Behaviour only | `m0-accept` 1 |
| 0 missed deadlines in 10 min at full load | `Diag.missed_deadlines`, delta over the run | – | `m0-accept` 1 |
| Jitter p99 < 0.5 ms, max reported | `LoopStats`, 10 µs bins | – | `m0-accept` 1 |
| max(exec) + max(jitter) < 5 ms | Margin = 5000 µs − (max exec + max jitter) | – | `m0-accept` 1 |
| ≥ 20% free RAM | Linker map, enforced in CI | **Pass**: 56.1% free (57,528 of 131,072 B used, incl. the 24 KB micro-ROS heap) | same image |
| No allocation in steady state | Static tasks, `_sbrk` refuses, `traceMALLOC` counter, CI rejects a linked `malloc` | **Pass** (0 after start) | `Diag` |
| Supervisor detects a hung control task | Unit tests, POSIX `TestHook` 2, real IWDG | **Pass** | `m0-accept` 3 |
| IWDG not refreshed without valid control cycles | Unit tests (stall, persistent overrun), POSIX | **Pass** | `m0-accept` 3 |
| Reset never enables the servos | OE high before any other init, FAULT `WATCHDOG_RESET` at boot, Diag OE bits | **Pass** | `m0-accept` 3 |

## Watchdog and supervisor timing

The IWDG runs from the LSI (17–47 kHz over temperature and parts), prescaler 32, reload 50: timeout **34 ms at
worst (fast LSI), 94 ms at best**. Every margin below is taken against 34 ms.

**What the supervisor checks.** A bare heartbeat never counts. A control cycle is *valid* when it finished within
5 ms of its own release and its release follows the previous one by a whole number of periods (no repeated,
backwards or off-grid release). The test runs inside `control_completed()` for every cycle, so a late cycle is
caught even if the next one completes before the supervisor's next 1 ms step (a gap the first version had; now
covered by `test_late_cycle_hidden_by_next`). Separately, the supervisor checks on its own clock that the next
completion arrives by the next deadline (stall).

**When it refreshes the IWDG** (every 1 ms step, all must hold):

| Condition | If not |
| --- | --- |
| A valid completion arrived by the next cycle's deadline (release + 10 ms) | Stall: latch, OE off, refresh stops → reset |
| Fewer than 20 invalid cycles in a row (100 ms) | Persistently late: refresh stops → reset |
| During boot: first cycle within 500 ms of start | Latch, refresh stops → reset |

One invalid cycle latches `CONTROL_DEADLINE` and cuts OE in the same step, but refreshing continues: the MCU stays
up to report the fault and recover through ClearFault. A reset would add nothing once the outputs are off.

**Worst cases against 34 ms:**

| Quantity | Value | Margin to 34 ms |
| --- | --- | --- |
| Refresh gap, healthy | 1 ms + supervisor wake-latency spread (expected < 50 µs); measured as `Diag.wdt_gap_us_max` | ≥ 30× |
| Supervisor scheduling latency | Highest-priority task, woken by the tick; only ISRs (UART/DMA, HAL tick, all short) and kernel critical sections can delay it. No code masks interrupts for long, there are no flash writes, the IWDG freezes when the debugger halts the core. Measured as `Diag.sup_jitter_us_max` | – |
| Hang → refresh stops | ≤ 10 ms after the last valid release (+ 1 ms step) | – |
| Hang → reset | 10–11 ms + 34–94 ms ≈ 45–105 ms. OE is already off from the first missed deadline (≤ 1 ms) | – |
| Late cycle → OE off | ≤ 1 ms after the missed deadline (+ wake latency); measured as `Diag.latch_delay_us` | – |

`m0-accept` fails if the measured refresh gap exceeds 2 ms (a 17× margin) or the latch comes later than 1.1 ms
after the deadline.

## Board acceptance procedure

Black Pill with the A+ boards wired (servos may stay unplugged: M0 drives no actuator) and the `m0test` image.

**USB-UART adapter, check before connecting:**
- Logic level really 3.3 V. Many CH340/CP2102 boards have a 5 V/3.3 V jumper or ship 5 V TX: measure the adapter's
  idle TX pin with a meter (3.3 V, not 5 V). PA10 tolerates 5 V, but don't rely on that.
- Common GND between the adapter and the Black Pill.
- Cross TX/RX: adapter TX → PA10 (RX), adapter RX ← PA9 (TX). Do not power the Black Pill from the adapter's 5 V
  pin while it is also on USB.

```powershell
cmake -S firmware\stm32f411 -B firmware\stm32f411\build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build firmware\stm32f411\build
STM32_Programmer_CLI -c port=SWD -w firmware\stm32f411\build\mico_mcu_m0test.hex -v -rst
pip install pyserial
python firmware\stm32f411\tools\mico_link.py --port COM5 m0-accept --minutes 10 --report m0_accept.json
```

1. **Full load, 10 min**: Enable; both PCA9685 at 50 Hz; all sensors; `State` 100 Hz; trajectories streamed and
   paced by the MCU's clock; `Hello` at 20 Hz.
2. **Overrun**: one cycle stretched by 8 ms. Expected: latch and OE off within 1 ms of the deadline, no reset,
   ClearFault → IDLE with OE still off, Enable works again.
3. **Hang**: the control task spins forever. Expected: IWDG reset, boot in FAULT `WATCHDOG_RESET`, reset cause IWDG,
   OE off, Enable refused, ClearFault → IDLE with OE still off.

The same command was run end to end against the simulated MCU behind a pty (`make -C
firmware/stm32f411/test/posix accept-sim`, also in CI): it checks the tool and the sequence, not the timing.

## m0_accept.json

Raw numbers are kept next to every PASS/FAIL, plus every 1 Hz `Diag` sample of the run (`diag_samples`), so later
milestones (micro-ROS, more CPU load) can be compared against this baseline.

| Metric | JSON field |
| --- | --- |
| Control frequency | `control.hz_mcu_clock`, `control.hz_host_clock`, cycle counts |
| Deadline | `deadline.missed_in_run`, `deadline.missed_since_boot` |
| Execution | `execution_us.avg`, `.p99`, `.max` |
| Jitter | `jitter_us.p99`, `.max` |
| Timing margin | `timing_margin_us` = 5000 − (max exec + max jitter) |
| Supervisor | `supervisor.wake_jitter_us_max`, `supervisor.watchdog_refresh_gap_us_max` |
| Watchdog | `fault_injection.hang.reset_cause` |
| Fault injection, detection | `fault_injection.overrun.latch_after_deadline_us` (MCU), `.host_observed_ms`; `fault_injection.hang.back_after_ms` (host: detection + IWDG + boot) |
| Servo output | `fault_injection.overrun.oe_after`, `fault_injection.hang.oe_after_reset`, `.oe_after_clear` |
| Resources | `cpu_permille`, `stack_free_words`, `heap_bytes`, `allocs_after_start`, `link` |

The MCU cannot report its own hang detection time across the reset; the host-side `back_after_ms` bounds it.

## What was built

| Item | Where |
| --- | --- |
| CubeMX-layout project, CMake + arm-none-eabi-gcc 13.2, release and `m0test` images | `firmware/stm32f411/` |
| HAL F4 v1.8.6, CMSIS-Device F4 v2.6.9, CMSIS 5.9.0, FreeRTOS-Kernel V11.1.0, pinned by commit SHA | `tools/fetch_drivers.py`, `Drivers/VERSIONS.md`, `--check` in CI |
| Four static tasks: supervisor, control, comm, diagnostics | `app/tasks.cpp` |
| Supervisor: valid-cycle test, stall check, OE ownership, watchdog policy, latch delay | `lib/core/safety/supervisor.*` |
| Control cycle coupling controller and supervisor | `lib/core/controller/control_cycle.*` |
| Loop statistics: exec (avg, p99, max), jitter (p99, max), missed | `lib/core/timing/loop_stats.*` |
| Lock-free SPSC byte ring, fence-free seqlock (TSan-tested) | `lib/core/link/` |
| UART: circular RX DMA with lap and overrun accounting, error restart; TX DMA from a private buffer | `hardware/board_stm32.cpp` |
| `Diag` (1 Hz, 86 bytes) with supervisor timing and OE state; `TestHook` (acceptance image only) | `tools/gen_tables.py` |
| ClearFault refused while the cause is present | `lib/core/controller/controller.cpp` |
| POSIX integration test; pty mode with a simulated IWDG for the host tool | `firmware/stm32f411/test/posix/` |
| `diag` and `m0-accept` commands | `tools/mico_link.py` |

## Host results

| Suite | Checks | Result |
| --- | --- | --- |
| `test_core` (regression, ASan + UBSan) | 361 | pass |
| `test_safety` (supervisor, valid cycles, stall, persistent overrun, latch delay, clock wrap, OE, clear, statistics) | 4129 | pass |
| `test_threads` (ring and seqlock on two real threads, TSan) | 2,000,000 bytes | pass |
| POSIX integration, normal / after watchdog reset | 23 / 6 | pass; 32 of 32 runs with the host 2× oversubscribed |
| `m0-accept` against the simulated MCU | 22 | pass |

The POSIX port runs on simulated time that follows the host's scheduling. Under heavy host overload (12×) the
simulated control task really does miss deadlines, and the supervisor latches them as it should; the scripted
checks are therefore run on an unloaded runner. Timing values come from the board only.

## Memory (release image)

| | Bytes | Of |
| --- | --- | --- |
| Flash | 34,608 | 512 KB (6.6%) |
| RAM, total | 57,528 | 128 KB (43.9%), 73,544 B free |
| of which FreeRTOS heap (micro-ROS, M1) | 24,576 | 0 allocations after start in M0 |
| of which task stacks | 8,704 + 512 idle | supervisor 1 KB, control 4 KB, comm 1.5 KB, diagnostics 2 KB |
| of which rings | 5,656 | RX 1 KB, TX 4 KB, Diag 512 B |
| of which MSP stack | 2,048 | start-up and interrupts |

## Timing estimate (replaced by the board numbers)

Polled HAL transfers at 400 kHz I²C (22.5 µs per byte), ADC at 24 MHz with 84-cycle sampling:

| Step | Every cycle | Cycle 0 mod 4 | Cycle 2 mod 4 |
| --- | --- | --- | --- |
| Mux: 16 channels × (5 µs settle + 2 conversions) | 260 µs | | |
| NTC × 2 | 25 µs | | |
| MPU6050, 14 bytes | 390 µs | | |
| INA226 × 2, 4 registers (every 2nd cycle) | | 490 µs | 490 µs |
| PCA9685 legs, 12 channels in one transfer | | 1,125 µs | |
| PCA9685 arms, 8 channels | | | 765 µs |
| Controller, State encoding | ~100 µs | | |
| **Total** | **~0.8 ms** | **~2.4 ms** | **~2.0 ms** |

Expected margin about 2.5 ms. If the board shows the polled I²C matters, move the PCA9685 writes to I²C DMA
(`board_stm32.cpp` only).

## Decisions (reviewed)

| # | Item | Decision |
| --- | --- | --- |
| 1 | No `.ioc` yet (CubeMX GUI not available here) | Accepted for M0. Generate and commit the `.ioc` before the project grows further |
| 2 | USB CDC removed, USART1 only | Accepted: matches micro-ROS over UART |
| 3 | IWDG ~50 ms, gated by the supervisor | Accepted with the condition that worst-case timing is shown against 34 ms: analysis above, measured by `m0-accept` |
| 4 | micro-ROS SHA and agent image digest pinned in M1 | Accepted: pin only after verifying the Jazzy image upstream |

## Conditions carried into M1

- The STM32 is the only safety authority. micro-ROS may request Enable or ClearFault; it never drives OE, never
  bypasses the supervisor, and a request is refused while its cause is present.
- `mico_msgs`: compact fixed-size embedded messages, sizes checked against the real CDR; full ROS messages stay on
  the Radxa.
- Timestamped trajectories into a jitter buffer; the control task never executes ROS callbacks directly. Reject
  trajectories when clock sync is lost or the offset is too large; expired trajectories fall back to the safe state.
- Control / supervisor separation as in M0. ROS packages under `src/`; `lib/core` stays ROS-free.
- STM32CubeMX + FreeRTOS + UART circular DMA.
- UART utilisation ≤ 50% with micro-ROS, and the 40 ms latency budget, both measured (M0 has no micro-ROS, so
  neither can be shown yet).
- Re-run `m0-accept` with micro-ROS loaded and compare against the M0 `m0_accept.json`.
