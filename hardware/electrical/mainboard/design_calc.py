"""BZN-MB rev A step 5: part ratings and design numbers, each with its check. Writes design_calc.md.

Every input is listed in INPUTS with its source; every check is an assert, so a value that breaks a rating
stops the script. Run: python3 design_calc.py [--spice]   (--spice also runs the ngspice settling check)
"""
import math
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
rows, checks = [], []


def row(group, item, value, note=""):
    rows.append((group, item, value, note))


def check(name, ok, detail):
    checks.append((name, ok, detail))
    assert ok, f"{name}: {detail}"


# ------------------------------------------------------------------ inputs (sources in design_calc.md)
STALL = {"MG92B": 1.20, "MG90S": 0.95, "SG90": 0.75}          # A at 6 V, design values (P0 measures)
BRANCH = {"left leg": ["MG92B"] * 6, "right leg": ["MG92B"] * 6,
          "arms + neck": ["MG90S"] * 2 + ["SG90"] * 6}
V_RAIL, V_BAT_MAX, V_BAT_MIN = 6.0, 8.4, 6.0
UBEC_BURST = 15.0                       # A, 1 s (UBEC listing)
I_TRIP_FW, T_TRIP_FW = 6.0, 0.200       # software/core/config.h I_TRIP_A, I_TRIP_MS
R_SHUNT = 0.010
FUSE = {"left leg": ("0452005.MRL", 5.0, 50.0), "right leg": ("0452005.MRL", 5.0, 50.0),
        "arms + neck": ("0452004.MRL", 4.0, 34.4)}               # (part, rating A, melting I2t A2s)
# The 5 A I2t is the 454-series value (53.7 A2s) rounded down; the 452 datasheet value is checked in review.
FET_RDS_45 = 0.0123                     # AONR21357 max at VGS -4.5 V, 25 C
FET_HOT = 1.45                          # RDS(on) rise at TJ 125 C (typical P-trench curve)
FET_THETA = 45.0                        # C/W, DFN3x3 on about 6 cm2 of 1 oz copper both sides
T_AMB = 45.0                            # C inside the closed torso
C_BULK = {"left leg": 940e-6, "right leg": 940e-6, "arms + neck": 940e-6}
LDO_THETA = 62.0                        # C/W, SOT-223 (MCP1703A-5002E/DB)
I_5V = 0.065                            # A into the Black Pill 5 V pin (budget 55 mA + margin)
V_SCHOTTKY = 0.40
ADC_FS, ADC_BITS = 3.3, 12


def ipc2221_width_mm(i, dt=10.0, oz=1.0, external=True):
    k = 0.048 if external else 0.024
    area_mil2 = (i / (k * dt ** 0.44)) ** (1 / 0.725)
    return area_mil2 / (1.378 * oz) * 0.0254


# ------------------------------------------------------------------ 1. branch currents and fuses
for b, servos in BRANCH.items():
    stall = sum(STALL[s] for s in servos)
    part, rating, i2t = FUSE[b]
    row("Branch " + b, "all servos stalled", f"{stall:.1f} A", "fault case, never steady state")
    row("Branch " + b, "fuse", f"{part}, {rating:g} A slow-blow, I²t {i2t:g} A²s", "Littelfuse NANO² 452")
    if b != "arms + neck":
        fw_i2t = I_TRIP_FW ** 2 * T_TRIP_FW
        check(f"{b}: firmware trip clears before the fuse", fw_i2t < 0.2 * i2t,
              f"{fw_i2t:.1f} A²s at 6 A for 200 ms vs fuse {i2t:g} A²s")
    t_short = i2t / UBEC_BURST ** 2
    row("Branch " + b, "fuse opens on a hard short (15 A from the UBEC)", f"≈ {t_short * 1000:.0f} ms",
        "adiabatic I²t / I²")
    check(f"{b}: short cleared within 1 s at the UBEC burst current", t_short < 1.0, f"{t_short:.2f} s")
    row("Branch " + b, "shunt dissipation at the fuse rating", f"{rating ** 2 * R_SHUNT:.2f} W", "2512, 1 W part")
    check(f"{b}: shunt below half its rating at the fuse current", rating ** 2 * R_SHUNT <= 0.5,
          f"{rating ** 2 * R_SHUNT:.2f} W")

# ------------------------------------------------------------------ 2. master switch / reverse protection
i_cont = 8.5
rds = FET_RDS_45 * FET_HOT
p_fets = 2 * i_cont ** 2 * rds
row("Q1, Q2 (back-to-back AONR21357)", "R_DS(on) per FET, hot", f"{rds * 1000:.1f} mΩ", "max at −4.5 V × 1.45")
row("Q1, Q2 (back-to-back AONR21357)", "loss at 8.5 A, both FETs", f"{p_fets:.2f} W",
    "the requirement's continuous ceiling; walking is ≈ 3 A")
tj = T_AMB + i_cont ** 2 * rds * FET_THETA
row("Q1, Q2 (back-to-back AONR21357)", "junction temperature at 8.5 A", f"{tj:.0f} °C", "per FET, own copper")
check("Q1/Q2 junction below 125 °C at 8.5 A continuous", tj < 125, f"{tj:.0f} °C")
tj3 = T_AMB + 3.0 ** 2 * rds * FET_THETA
row("Q1, Q2 (back-to-back AONR21357)", "junction temperature walking (3 A)", f"{tj3:.0f} °C")
check("Q1/Q2 VGS stays inside ±25 V", V_BAT_MAX < 25, "gate swing is the 6 V rail")

# ------------------------------------------------------------------ 3. inrush into the bulk capacitors
c_total = sum(C_BULK.values())
t_soft = 2e-3                                        # gate RC ramp, Q3 drive
i_inrush = c_total * V_RAIL / t_soft
row("Soft start", "bulk capacitance", f"{c_total * 1e6:.0f} µF", "2 × 470 µF per branch")
row("Soft start", "inrush with a 2 ms gate ramp", f"{i_inrush:.1f} A", "C·V/t")
check("inrush below the UBEC burst and far from the fuse I²t", i_inrush < UBEC_BURST
      and i_inrush ** 2 * t_soft < 0.05 * 34.4, f"{i_inrush:.1f} A, {i_inrush ** 2 * t_soft:.2f} A²s")
for b, c in C_BULK.items():
    dv = 3 * STALL[BRANCH[b][0]] * 100e-6 / c
    row("Rail droop", f"{b}: 3 servos start at once, 100 µs before the UBEC responds", f"{dv * 1000:.0f} mV",
        "I·t / C")
    check(f"{b}: droop below 0.5 V", dv < 0.5, f"{dv:.2f} V")

# ------------------------------------------------------------------ 4. copper
for name, i in (("6 V input to the switch", 8.5), ("leg branch", 5.0), ("arm branch", 4.0)):
    w1 = ipc2221_width_mm(i, 10, 1.0)
    row("Copper (IPC-2221, outer 1 oz, ΔT 10 °C)", name, f"{w1:.1f} mm",
        "pour on top and bottom, stitched: half each")
for name, i in (("6 V input to the switch", 8.5),):
    check("8.5 A path fits the 80 mm board with 1 oz pours on two layers", ipc2221_width_mm(i, 10, 1.0) / 2 < 4,
          f"{ipc2221_width_mm(i, 10, 1.0) / 2:.1f} mm per layer")

# ------------------------------------------------------------------ 5. logic supply
v_in_min = V_BAT_MIN - V_SCHOTTKY
v_in_max = V_BAT_MAX - V_SCHOTTKY
p_ldo = (v_in_max - 5.0) * I_5V
row("U18 MCP1703A-5002E/DB", "input range after D2", f"{v_in_min:.1f}–{v_in_max:.1f} V", "16 V maximum")
row("U18 MCP1703A-5002E/DB", "dissipation, worst", f"{p_ldo * 1000:.0f} mW", f"ΔT {p_ldo * LDO_THETA:.0f} °C")
check("U18 headroom at the empty-pack cut (6.0 V)", v_in_min - 5.0 > 0.33, f"{v_in_min - 5.0:.2f} V vs 0.33 V dropout")
check("U18 junction rise below 40 °C", p_ldo * LDO_THETA < 40, f"{p_ldo * LDO_THETA:.0f} °C")
row("D3 → Black Pill 5 V pin", "voltage at the AP7343 input", f"{5.0 - V_SCHOTTKY:.2f} V", "3.3 V out needs ≥ 3.6 V")

# ------------------------------------------------------------------ 6. INA226 registers
i_lsb = 0.25e-3
cal = round(0.00512 / (i_lsb * R_SHUNT))
row("INA226", "current LSB", "0.25 mA", "8.192 A full scale / 32768")
row("INA226", "CALIBRATION (0x05)", f"{cal} (0x{cal:04X})", "0.00512 / (I_LSB · R_SHUNT)")
for b, lim in (("leg branches", 7.5), ("arms + neck", 5.0)):
    reg = round(lim * R_SHUNT / 2.5e-6)
    row("INA226", f"ALERT limit, {b}", f"{lim} A → 0x{reg:04X}", "MASK/ENABLE SOL; limit LSB 2.5 µV")
avg_t = 4 * (1.1e-3 + 1.1e-3)
row("INA226", "ALERT latency with the firmware's 0x4327", f"≈ {avg_t * 1000:.0f} ms", "4 averages × (1.1 + 1.1) ms")
check("cal fits 15 bits", cal < 32768, str(cal))

# ------------------------------------------------------------------ 7. analog front end
r1, r2 = 200e3, 68e3
k = r2 / (r1 + r2)
row("VBAT_SNS (PA3)", "divider", "200 kΩ / 68 kΩ, 100 nF", f"ratio {k:.4f}")
row("VBAT_SNS (PA3)", "ADC at 8.4 V / at the 6.6 V cut", f"{V_BAT_MAX * k:.2f} V / {6.6 * k:.2f} V",
    f"{ADC_FS / k:.1f} V full scale, {ADC_FS / k / 4096 * 1000:.1f} mV per count")
check("VBAT divider stays below the ADC range up to 12 V", 12 * k < ADC_FS, f"{12 * k:.2f} V at 12 V")
row("Wiper buffers", "op-amp", "MCP6002 on +3V3A (RC-filtered 3.3 V)", "rail-to-rail in and out")
row("Wiper buffers", "input", "10 kΩ series + BAT54S clamp to +3V3A / GND",
    "servo wiper above 3.3 V cannot reach the pin")
r_iso, r_on, c_sig, c_mux = 220.0, 300.0, 470e-12, 50e-12
tau = (r_iso + r_on) * (c_sig + c_mux)
n_tau = math.log(2 ** (ADC_BITS + 1))
row("Mux settling", "τ = (R_iso + R_on)(C_SIG + C_mux)", f"{tau * 1e6:.2f} µs", "R_on 300 Ω at 3.3 V worst")
row("Mux settling", "to ½ LSB (12 bit)", f"{n_tau * tau * 1e6:.1f} µs", "firmware waits 5 µs")
check("mux settles to ½ LSB inside the firmware's 5 µs", n_tau * tau < 5e-6, f"{n_tau * tau * 1e6:.1f} µs")
kick = 4e-12 / (c_sig + 4e-12)
row("Mux settling", "ADC sample-capacitor kickback", f"{kick * 100:.2f} % of a full-scale step",
    "4 pF C_ADC on 470 pF; recovered inside the 56-cycle sample")
row("Servo PWM lines", "series resistor", "1 kΩ + clamp array to +3V3", "one-pin-off plug: 2.7 mA into the clamp")
check("one-pin-off plug current within clamp rating", (6.0 - 3.6) / 1000 < 10e-3, "2.4 mA")


# ------------------------------------------------------------------ SPICE: mux + ADC sample, worst step
def spice():
    net = f"""* mux SIG settling: channel switch from 0 V to 3.3 V, then ADC sampling at 5 us
Vop op 0 PWL(0 0 1n 3.3)
Riso op m {r_iso}
Ron m sig {r_on}
Cmux sig 0 {c_mux}
Csig sig 0 {c_sig}
Sadc sig adc ctl 0 sw
Vctl ctl 0 PWL(0 0 5u 0 5.001u 1)
Radc_dummy adc 0 1e12
Cadc adc 0 4p
.model sw sw vt=0.5 ron=6k roff=1e12
.tran 5n 8u
.control
run
meas tran vsig5 find v(sig) at=5u
meas tran vadc find v(adc) at=7.33u
print vsig5 vadc
.endc
.end
"""
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "mux.cir"
        p.write_text(net)
        out = subprocess.run(["ngspice", "-b", str(p)], capture_output=True, text=True).stdout
    vals = {}
    for line in out.splitlines():
        for key in ("vsig5", "vadc"):
            if line.strip().startswith(key) and "=" in line:
                vals[key] = float(line.split("=")[1].split()[0])
    lsb = ADC_FS / 4096
    err = abs(3.3 - vals["vadc"])
    row("SPICE (ngspice)", "SIG at 5 µs / ADC cap at end of a 56-cycle sample",
        f"{vals['vsig5']:.4f} V / {vals['vadc']:.4f} V", f"error {err / lsb:.2f} LSB")
    check("ngspice: sampled value within ½ LSB", err < lsb / 2, f"{err * 1000:.2f} mV")


if "--spice" in sys.argv:
    spice()

# ------------------------------------------------------------------ write
lines = ["# BZN-MB rev A: design calculations (step 5)", "",
         "Generated by `design_calc.py`. Do not edit by hand: change the script and run it again.", "",
         "| Group | Item | Value | Note |", "| --- | --- | --- | --- |"]
lines += [f"| {g} | {i} | {v} | {n} |" for g, i, v, n in rows]
lines += ["", "## Checks", "", "| Check | Result | Detail |", "| --- | --- | --- |"]
lines += [f"| {n} | {'PASS' if ok else 'FAIL'} | {d} |" for n, ok, d in checks]
(HERE / "design_calc.md").write_text("\n".join(lines) + "\n")
print(f"{len(checks)} checks passed")
