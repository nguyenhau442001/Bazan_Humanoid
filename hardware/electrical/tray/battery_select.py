"""Battery selection for leg v4: energy budget per operating mode, then each candidate pack against runtime,
fault current, the pelvis cradle envelope and the knee load its mass adds.

Servo currents come from the MuJoCo twin, as in power_calc.py (idle + stall current x torque / stall torque).
That model has no acceleration, back-EMF or servo hunting current, so every runtime is given twice: as modelled
and with the servo current x K_PESSIMISTIC. The logic budget is datasheet-typical and MEASURE.

    python3 hardware/electrical/tray/battery_select.py      # writes battery_select.md next to this file
"""
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(HERE))
import power_calc as pc  # noqa: E402

K_PESSIMISTIC = 1.5          # real hobby servos vs the torque-only current model (replace with the P0 measurement)
BUCK_EFF = 0.88              # 5 V buck for the logic
KNEE_LIMIT = 1 / 3           # knee p99 must stay under 1/3 stall (leg v4 rule)
BASE_PACK_G = 57.0           # the pack the CAD carries today (electronics.py BATT)

# logic, watts on the 5 V / 3.3 V side: (typical, peak, source)
LOGIC_W = {
    "Radxa Zero 3W (RK3566, Wi-Fi on)": (2.5, 5.0, "Radxa asks for a 5 V 2 A supply; typical under ROS load, MEASURE"),
    "IMX219 camera streaming": (0.25, 0.30, "module typical"),
    "1.9 in LCD + backlight": (0.15, 0.20, "ST7789 + LED backlight, typical"),
    "STM32F411 Black Pill, 100 MHz": (0.15, 0.20, "about 30 mA + LDO loss"),
    "2 x PCA9685, MPU6050, 4 x INA226, VL53L0X, mux, op-amps, mics": (0.20, 0.25, "datasheet supply currents"),
    "2 x UBEC no-load loss": (0.20, 0.20, "estimate, MEASURE"),
}

# pelvis cradle envelope (body.py): deck 0 top to deck 1 underside is SO_A = 40 mm; the strap slots at |y| 18.5 bound
# the width, and moving them out to |y| 20.6 (PCA9685 bays start at |y| 23.8) gives 41 mm. Length: deck x -49..40
# minus the two 2 mm end walls and clearance.
ENV_L, ENV_W, ENV_W_MOVED, ENV_H = 80.0, 37.0, 41.0, 37.0


@dataclass
class Pack:
    name: str
    chem: str            # "LiPo" pouch or "Li-ion" cylindrical (steel can)
    mah: float
    v_nom: float         # per cell
    cont_a: float        # continuous discharge the pack (cells or the label derated) can give
    mass_g: float
    dims: tuple          # L, W, H mm
    usable: float        # fraction of rated capacity above the firmware empty point (3.30 V/cell OCV)
    vnd: int             # estimate
    source: str


PACKS = [
    Pack("2S LiPo 1000 mAh 30C (BOM today)", "LiPo", 1000, 3.7, 15.0, 57.0, (56, 30, 19), 0.80, 155_000,
         "BOM v0.9; cheap 30C label derated x 0.5"),
    Pack("2S LiPo 1500 mAh 100C (Turnigy Graphene class)", "LiPo", 1500, 3.7, 75.0, 87.0, (70, 35, 19), 0.80,
         280_000, "HobbyKing listing: 87 g, 70 x 35 x 19 mm; label derated x 0.5"),
    Pack("2S1P 18650 Molicel P28A + 2S BMS 20 A", "Li-ion", 2800, 3.6, 20.0, 102.0, (68, 39, 20), 0.85, 300_000,
         "cells 46 g, 18.6 x 65.2 mm, 35 A max; +10 g nickel, BMS, shrink, leads; BMS limits to 20 A"),
    Pack("2S1P 18650 Sony/Murata VTC6 + 2S BMS 20 A", "Li-ion", 3000, 3.6, 15.0, 103.0, (68, 39, 20), 0.85, 300_000,
         "cells 46.6 g, 15 A without the 80 °C cut; +10 g as above"),
    Pack("2S2P 18650 Molicel P28A", "Li-ion", 5600, 3.6, 20.0, 195.0, (68, 39, 39), 0.85, 560_000,
         "four cells stacked two high"),
]


def knee_load(extra_g, cache={}):
    """Knee p99 / stall over the demo gait with extra_g added at the battery, from the MuJoCo twin."""
    key = round(extra_g)
    if key not in cache:
        from check_mujoco import mm
        import check_mujoco as cm
        sim = mm.BazanSim(mm.cad_mjcf())
        m = sim.model
        b = 1                                                       # base_link carries the battery
        m0, dm = m.body_mass[b], extra_g / 1000
        pt = np.array([-0.0045, 0.0, m.body_ipos[b][2]])            # body.py BATT_C, at the pelvis CoM height
        m.body_ipos[b] = (m0 * m.body_ipos[b] + dm * pt) / (m0 + dm)
        m.body_inertia[b] *= (m0 + dm) / m0
        m.body_mass[b] = m0 + dm
        r = cm.run(sim)
        cache[key] = (max(r["p99"]["l_knee"], r["p99"]["r_knee"]), r["fell"] is None)
    return cache[key]


def main():
    rows, checks = [], []
    walk = sum(pc.walking_currents().values())
    servo_w = {"Torque off (logic only)": 0.0, "Standing": stand_current() * pc.V_RAIL,
               "Walking (demo gait)": walk.mean() * pc.V_RAIL}
    logic_typ = sum(v[0] for v in LOGIC_W.values()) / BUCK_EFF
    logic_peak = sum(v[1] for v in LOGIC_W.values()) / BUCK_EFF

    def batt_w(mode, k=1.0):
        return servo_w[mode] * k / pc.UBEC_EFF + logic_typ

    # a typical bench session: a third each standing, walking and torque off
    session = {"Torque off (logic only)": 1 / 3, "Standing": 1 / 3, "Walking (demo gait)": 1 / 3}

    def session_w(k=1.0):
        return sum(f * batt_w(m, k) for m, f in session.items())

    i_fault = pc.FUSE["battery"][0] * 1.02           # every branch at its limit: the battery fuse must open
    for p in PACKS:
        wh = 2 * p.v_nom * p.mah / 1000 * p.usable
        t_walk = wh / batt_w("Walking (demo gait)") * 60
        t_walk_p = wh / batt_w("Walking (demo gait)", K_PESSIMISTIC) * 60
        t_sess = wh / session_w(K_PESSIMISTIC) * 60
        knee, upright = knee_load(p.mass_g - BASE_PACK_G)
        L, W, H = p.dims
        fit = "yes" if (L <= ENV_L and W <= ENV_W and H <= ENV_H) else (
            "slots moved" if (L <= ENV_L and W <= ENV_W_MOVED and H <= ENV_H) else "no")
        ok = dict(knee=knee < KNEE_LIMIT and upright, fault=p.cont_a >= i_fault, fit=fit != "no",
                  runtime=t_walk_p >= 15.0)
        rows.append((p, wh, t_walk, t_walk_p, t_sess, knee, fit, ok))
    ok_packs = [r for r in rows if all(r[7].values())]
    best = max(ok_packs, key=lambda r: r[3]) if ok_packs else None
    checks.append(("logic peak inside the 5 V 3 A buck", logic_peak * BUCK_EFF / 5.0 <= 3.0,
                   f"{logic_peak * BUCK_EFF / 5.0:.2f} A at 5 V"))
    checks.append(("logic budget at or under power_calc LOGIC_A", logic_peak / pc.V_BAT <= pc.LOGIC_A,
                   f"{logic_peak / pc.V_BAT:.2f} A vs {pc.LOGIC_A:g} A"))
    for n, ok, d in checks:
        assert ok, f"{n}: {d}"

    out = ["# Battery selection (leg v4)", "",
           "Generated by `hardware/electrical/tray/battery_select.py`; do not edit by hand. Servo currents are the "
           f"MuJoCo twin's (torque-only model); the pessimistic column multiplies them by {K_PESSIMISTIC:g} until "
           "the P0 bench measurement exists.", "",
           "## Logic budget", "", "| Load | Typical W | Peak W | Source |", "| --- | --- | --- | --- |"]
    out += [f"| {k} | {v[0]:.2f} | {v[1]:.2f} | {v[2]} |" for k, v in LOGIC_W.items()]
    out += [f"| **At the battery** (buck {BUCK_EFF * 100:.0f} %) | **{logic_typ:.2f}** | **{logic_peak:.2f}** | |",
            "", "## Power per mode, at the battery", "",
            "| Mode | Servo rail W | Battery W (model) | Battery W (pessimistic) | Battery A (pessimistic) |",
            "| --- | --- | --- | --- | --- |"]
    for m in servo_w:
        out.append(f"| {m} | {servo_w[m]:.1f} | {batt_w(m):.1f} | {batt_w(m, K_PESSIMISTIC):.1f} | "
                   f"{batt_w(m, K_PESSIMISTIC) / pc.V_BAT:.2f} |")
    out += [f"| Bench session (⅓ each) | | {session_w():.1f} | {session_w(K_PESSIMISTIC):.1f} | "
            f"{session_w(K_PESSIMISTIC) / pc.V_BAT:.2f} |", "",
            "## Candidates", "",
            f"Envelope in the pelvis cradle: L ≤ {ENV_L:g}, W ≤ {ENV_W:g} ({ENV_W_MOVED:g} with the strap slots moved "
            f"out), H ≤ {ENV_H:g} mm. Knee: p99 of stall over the demo gait with the pack's mass, must stay under "
            f"{KNEE_LIMIT * 100:.1f} %. Fault: the pack must hold {i_fault:.1f} A until the "
            f"{pc.FUSE['battery'][0]:g} A battery fuse opens.", "",
            "| Pack | Chem | Usable Wh | Walk min (model) | Walk min (pess.) | Session min (pess.) | Mass g | "
            "Knee p99 | Fits | Cont. A | ≈ VND | Pass |",
            "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for p, wh, tw, twp, ts, knee, fit, ok in rows:
        bad = ", ".join(k for k, v in ok.items() if not v)
        out.append(f"| {p.name} | {p.chem} | {wh:.1f} | {tw:.0f} | {twp:.0f} | {ts:.0f} | {p.mass_g:.0f} | "
                   f"{knee * 100:.1f} % | {fit} | {p.cont_a:g} | {p.vnd:,} | {'yes' if not bad else 'no: ' + bad} |")
    out += ["", "Sources: " + "; ".join(f"{p.name}: {p.source}" for p in PACKS) + ".", ""]
    out += [f"Best that passes everything: **{best[0].name}**." if best else "No candidate passes everything.", "",
            "## Checks", "", "| Check | Result | Value |", "| --- | --- | --- |"]
    out += [f"| {n} | {'pass' if ok else 'FAIL'} | {d} |" for n, ok, d in checks]
    (HERE / "battery_select.md").write_text("\n".join(out) + "\n")
    print("\n".join(out))


def stand_current():
    """Servo rail current holding the standing pose (A), from the MuJoCo twin."""
    from check_mujoco import mm
    sim = mm.BazanSim(mm.cad_mjcf())
    sim.reset()
    sim.step(1000)
    stall = np.abs(sim.model.actuator_forcerange[sim.act, 1])
    apn = np.array([pc.servo(j).stall_a / s for j, s in zip(mm.gait.JOINTS, stall)])
    amps = []
    for i in range(1000):
        sim.step()
        if i % 5 == 0:
            amps.append((pc.I_IDLE + apn * np.abs(sim.joint_efforts())).sum())
    return float(np.mean(amps))


if __name__ == "__main__":
    main()
