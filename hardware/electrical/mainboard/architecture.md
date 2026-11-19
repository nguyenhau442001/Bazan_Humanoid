# BZN-MB rev A: architecture (step 4)

Block diagram: [`block_diagram.svg`](block_diagram.svg) (source `block_diagram.dot`). Requirement IDs refer to
[`requirements.md`](requirements.md). Part numbers and values are fixed in step 5; this page fixes the structure.

## 1. Two power domains

| Domain | Source | Carries | Why separate |
| --- | --- | --- | --- |
| Servo rail 6 V | UBEC through the XT30 | up to 8.5 A continuous, 15 A for 1 s | A stall or a step pulls the rail down and puts amps of noise on it. |
| Logic and analog | VBAT lead, two LDOs | < 0.2 A | The MCU must keep running, and the ADC must keep reading, while the servo rail sags. |

**Servo rail (P-1, P-2, P-4).** XT30 in → reverse-polarity P-MOSFET → TVS → three branches. Each branch:
fuse → 10 mΩ shunt (Kelvin to its INA226) → bulk capacitor → servo headers. Left leg 6, right leg 6,
arms + neck 8. The PCA9685 V+ pins carry no servo current.

**Logic (P-5, P-7).** One wire from the battery lead (after the main fuse and switch) brings VBAT, 6.0–8.4 V,
to the board through a PTC and a series diode. Two LDOs, input rated ≥ 16 V:

- 5 V logic → Schottky → Black Pill 5 V pin. The Black Pill joins USB VBUS straight to its 5 V rail, so the
  Schottky keeps a plugged-in USB cable from back-feeding the board, and the board from back-feeding the PC.
  The Black Pill's own AP7343 (300 mA) makes 3.3 V for everything on the board; the budget below is under 100 mA.
- 5 V analog → op-amp supplies only.
- VBAT divider → PA3 (ADC1_IN3, free today). **This adds the battery under-voltage check the robot does not have
  yet:** the INA226s sit after the UBEC and read a regulated 6 V, which stays at 6 V until the pack is already
  over-discharged. A 2S pack is cut at 6.6 V (3.3 V a cell) under load.

The VBAT lead has no ground wire. Logic ground returns through the board's main ground, so a thin wire can
never become the return path for servo current.

## 2. Ground

- One ground plane (layer 2) under the whole board, split only by placement, not by cuts.
- Servo return current flows from the headers to the XT30 along the power side of the board. The analog side
  (op-amps, mux, dividers, the Black Pill's ADC pins) sits where no servo current passes underneath.
- Battery negative is the single star point: UBEC ground (thick), buck ground (Radxa) and the board meet there.
  The Radxa UART cable carries ground too; that loop is low-frequency and is accepted, with the cable kept short.

## 3. Protection layers (P-8)

| Layer | Acts on | Speed | Works when the MCU has hung? |
| --- | --- | --- | --- |
| Firmware rules | INA226 current, NTC, VBAT: hold pose, crouch, or OE off | 5–10 ms | no |
| Hardware trip (F-9) | INA226 ALERT (any branch over the limit) → latch → both OE high, servos limp | < 1 ms after the INA226 conversion | yes, once the INA226 limits were written at boot |
| Branch fuses | a shorted servo or cable | ms to s, by size | yes, always |
| Main fuse (battery lead) | a short anywhere after the battery, the UBEC and buck included | fastest at high current | yes, always |
| Reverse-polarity FET, PTC + diode | XT30 or VBAT lead connected backwards | immediate | yes |

The INA226 powers up with its alert disabled; the firmware writes the limit during `devices_init()`. The latch
powers up **tripped** (outputs off) and only the MCU clears it, so a board that never booted cannot drive a servo.
OE logic: each PCA9685 OE = MCU OE pin OR trip. MCU pins added: TRIP_CLEAR (output) and TRIP_STATE (input).

## 4. Logic budget (3.3 V from the Black Pill's AP7343, 300 mA)

| Load | Typical mA |
| --- | ---: |
| STM32F411 at 96 MHz, peripherals on | 30 |
| PCA9685 × 2 | 12 |
| INA226 × 3 | 1 |
| MPU6050 | 4 |
| CD74HC4067, latch, OR gates, I²C pull-ups (worst case) | 6 |
| Black Pill LEDs | 2 |
| **Total** | **≈ 55** |

The op-amps (6 × LMV358, ≈ 3 mA total) run from the analog 5 V LDO.

## 5. Signal paths

- **PWM:** PCA9685 → 220 Ω series → servo header signal pin (S-4).
- **Pots:** wiper → JST → LMV358 follower → (divider to 3.3 V if P0 shows wipers above it) → mux ch 0–11.
- **FSR:** divider with a fixed resistor → mux ch 12–15. **NTC:** divider → PA1, PA2. Every ADC input: series R
  and clamps to the 3.3 V rail (S-2).
- **Mux:** SIG → RC sized to settle inside the 5 µs the firmware waits → PA0. PA0 also has the Black Pill's
  KEY button to ground: pressing it while running reads zero on every mux channel, harmless behind the series R.
- **UART to the Radxa:** PA9/PA10, 3.3 V both sides, JST-GH with ground.
- **SWD:** 2.54 mm header with NRST, plus 0 Ω links to the Radxa connector so the Radxa can later flash the
  STM32 over SWD (OpenOCD on Linux GPIO) without opening the torso. Fitted only if wanted.

## 6. Decisions taken here

1. Logic power from VBAT, not from the 6 V servo rail (brown-out immunity).
2. Battery under-voltage measured directly on PA3.
3. Hardware trip latched, powering up tripped.
4. UBEC and buck stay modules in rev A; the main fuse and switch stay on the battery lead.
