"""Tray step 4: servo power for leg v4 (DS3218 / DS3225). Currents, firmware trip, fuses, UBECs, wire, battery.

Every input is named with its source; every rule is an assert, so a value that breaks one stops the script.
Walking currents come from the MuJoCo twin (the CAD URDF and the gait that runs on the robot): each servo draws
its idle current plus its stall current scaled by torque / stall torque. That model has no acceleration or
back-EMF current, so the measured P0 / P1 currents replace it as soon as they exist.

    python3 hardware/electrical/tray/power_calc.py      # writes power_calc.md next to this file
"""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "hardware" / "mechanical"))
sys.path.insert(0, str(REPO / "software" / "ai" / "sim" / "mujoco"))
import hardware as hw  # noqa: E402

# ------------------------------------------------------------------ inputs
V_RAIL = 6.0                       # servo rail, UBEC output
V_BAT = 7.4                        # 2S nominal
UBEC_A, UBEC_BURST_A, UBEC_EFF = 8.0, 15.0, 0.90   # Hobbywing 6 V 8 A UBEC listing: continuous, 1 s burst, efficiency
I_IDLE = 0.01                      # A per servo holding no load
I_TRIP, T_TRIP = 6.0, 0.200        # software/core/config.h I_TRIP_A, I_TRIP_MS (per leg branch)
INA_RANGE = 0.08192 / 0.010        # INA226 full scale 81.92 mV on the R010 shunt
FUSE = {"left leg": (5.0, 50.0), "right leg": (5.0, 50.0), "arms + neck": (4.0, 34.4), "battery": (15.0, 400.0)}
# (rating A, melting I2t A2s): slow-blow mini blade; the battery fuse I2t is the 15 A ATM typical value
LOGIC_A = 0.95                     # A from the battery at the logic peak (battery_select.py LOGIC_W)
PACK_MAH, PACK_C, USABLE = 1000.0, 30.0, 0.80      # BOM pack, its C rating, depth of discharge used
WIRE = {"22 AWG": 7.0, "20 AWG": 11.0, "18 AWG": 16.0}   # A, silicone wire in free air, short runs (chassis rating)

LEG = ["hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll"]
ARMS = ["shoulder_pitch", "shoulder_roll", "elbow"]


def servo(joint):
    key = joint[2:] if joint[:2] in ("l_", "r_") else joint
    return hw.SERVOS[hw.JOINT_SERVO[key]]


def branches():
    return {"left leg": [f"l_{j}" for j in LEG], "right leg": [f"r_{j}" for j in LEG],
            "arms + neck": [f"{s}_{j}" for s in "lr" for j in ARMS] + ["neck_yaw", "head_pitch"]}


def walking_currents():
    """Branch currents over the demo (crouch, 8 steps, 60° turn, wave) in the MuJoCo twin, sampled at 200 Hz."""
    from check_mujoco import mm
    gait = mm.gait
    sim = mm.BazanSim(mm.cad_mjcf())
    sim.reset()
    sim.step(1000)
    t, p = gait.trajectory_points(50.0, 2.0)
    sim.set_trajectory(gait.JOINTS, t, p)
    stall = np.abs(sim.model.actuator_forcerange[sim.act, 1])
    amps_per_nm = np.array([servo(j).stall_a / s for j, s in zip(gait.JOINTS, stall)])
    idx = {b: [gait.JOINTS.index(j) for j in js] for b, js in branches().items()}
    rec = {b: [] for b in idx}
    n = int((2.0 + gait.T_END + 1.0) / sim.model.opt.timestep)
    for i in range(n):
        sim.step()
        if i % 5 == 0:
            amps = I_IDLE + amps_per_nm * np.abs(sim.joint_efforts())
            for b, ix in idx.items():
                rec[b].append(amps[ix].sum())
    return {b: np.array(v) for b, v in rec.items()}


def main():
    rows, checks = [], []

    def check(name, ok, detail):
        checks.append((name, ok, detail))
        assert ok, f"{name}: {detail}"

    walk = walking_currents()
    total = sum(walk.values())
    stats = {b: dict(mean=v.mean(), rms=np.sqrt((v ** 2).mean()), p99=np.percentile(v, 99), peak=v.max())
             for b, v in list(walk.items()) + [("all servos", total)]}
    stalled = {b: sum(servo(j).stall_a for j in js) for b, js in branches().items()}

    # firmware trip: above anything the gait does, inside the INA226 range, cleared well before the fuse
    for b in ("left leg", "right leg"):
        s = stats[b]
        check(f"{b}: trip {I_TRIP:g} A at least 2 x the walking peak", I_TRIP >= 2 * s["peak"],
              f"peak {s['peak']:.2f} A")
        check(f"{b}: trip inside the INA226 range", I_TRIP < 0.9 * INA_RANGE, f"range {INA_RANGE:.2f} A")
        rating, i2t = FUSE[b]
        check(f"{b}: firmware trip clears before the fuse melts", I_TRIP ** 2 * T_TRIP < 0.2 * i2t,
              f"{I_TRIP ** 2 * T_TRIP:.1f} A²s vs fuse {i2t:g} A²s")
        check(f"{b}: fuse rating at least 2 x the walking rms", rating >= 2 * s["rms"], f"rms {s['rms']:.2f} A")
        t_open = i2t / stalled[b] ** 2
        rows.append((b, f"{stalled[b]:.2f} A", f"{s['mean']:.2f}", f"{s['p99']:.2f}", f"{s['peak']:.2f}",
                     f"{rating:g} A", f"≈ {t_open * 1000:.0f} ms"))
    s = stats["arms + neck"]
    rating, i2t = FUSE["arms + neck"]
    check("arms + neck: fuse rating at least 2 x the walking rms", rating >= 2 * s["rms"], f"rms {s['rms']:.2f} A")
    rows.append(("arms + neck", f"{stalled['arms + neck']:.2f} A", f"{s['mean']:.2f}", f"{s['p99']:.2f}",
                 f"{s['peak']:.2f}", f"{rating:g} A", f"≈ {i2t / stalled['arms + neck'] ** 2 * 1000:.0f} ms"))

    # UBECs, single fault: one branch at its limit (a leg at the firmware trip, the arms at their fuse rating) while
    # the rest walk. The left leg's UBEC also feeds the arms and neck.
    one = max(I_TRIP + stats["arms + neck"]["peak"], FUSE["arms + neck"][0] + stats["left leg"]["peak"])
    check("one UBEC per leg, single fault, inside the continuous rating", one <= UBEC_A,
          f"{one:.1f} A vs {UBEC_A:g} A")
    shared = I_TRIP + stats["right leg"]["peak"] + stats["arms + neck"]["peak"]
    single_ok = shared <= UBEC_A
    # a double fault (a leg jammed while the arms are stalled) is cut by the leg trip within 200 ms, inside the
    # UBEC's 1 s burst rating
    check("double fault inside the UBEC burst rating until the leg trip clears it",
          I_TRIP + FUSE["arms + neck"][0] <= UBEC_BURST_A, f"{I_TRIP + FUSE['arms + neck'][0]:.1f} A")
    check("walking peak, all servos, inside one UBEC", stats["all servos"]["peak"] <= UBEC_A,
          f"{stats['all servos']['peak']:.2f} A")

    # battery
    def batt(servo_a):
        return servo_a * V_RAIL / (UBEC_EFF * V_BAT) + LOGIC_A

    i_walk, i_peak = batt(stats["all servos"]["mean"]), batt(stats["all servos"]["peak"])
    i_worst = batt(max(I_TRIP + stats["right leg"]["peak"], FUSE["arms + neck"][0] + stats["left leg"]["peak"])
                   + stats["arms + neck"]["peak"])                      # single fault, as for the UBECs
    i_multi = batt(2 * I_TRIP + FUSE["arms + neck"][0])                 # everything at its limit: the battery fuse
    runtime = PACK_MAH / 1000 * USABLE / i_walk * 60
    check("pack C rating covers every branch at its limit", i_multi <= PACK_C * PACK_MAH / 1000,
          f"{i_multi:.1f} A vs {PACK_C * PACK_MAH / 1000:.0f} A")
    check("battery fuse holds a single fault", i_worst <= FUSE["battery"][0], f"{i_worst:.1f} A")
    check("battery INA226 sees the walking peak unclipped", i_peak < INA_RANGE, f"{i_peak:.2f} A")

    # wire: smallest gauge whose rating covers the fuse
    gauge = {b: next(g for g, a in WIRE.items() if a >= FUSE[b][0]) for b in FUSE}

    out = ["# Servo power for leg v4", "",
           "Generated by `hardware/electrical/tray/power_calc.py`; do not edit by hand. Walking currents come from "
           "the MuJoCo twin with each servo at idle current + stall current × torque / stall torque; P0 and P1 "
           "measurements replace them.", "",
           "## Servo stall currents (design values, `hardware.py` `stall_a`, MEASURE)", "",
           "| Servo | Stall torque at 6 V | Stall current | Joints |", "| --- | --- | --- | --- |"]
    for name in sorted({hw.JOINT_SERVO[k] for k in hw.JOINT_SERVO}):
        t = hw.SERVOS[name]
        js = ", ".join(k for k, v in hw.JOINT_SERVO.items() if v == name)
        out.append(f"| {name} | {t.stall_kgcm:g} kg·cm | {t.stall_a:g} A | {js} |")
    out += ["", "## Branches", "",
            "| Branch | All servos stalled | Walking mean (A) | Walking p99 (A) | Walking peak (A) | Fuse | "
            "Fuse opens with all stalled |", "| --- | --- | --- | --- | --- | --- | --- |"]
    out += [f"| {' | '.join(r)} |" for r in rows]
    a = stats["all servos"]
    out += ["", f"All servos together while walking: mean {a['mean']:.2f} A, p99 {a['p99']:.2f} A, "
            f"peak {a['peak']:.2f} A.", "",
            f"Firmware trip per leg branch: {I_TRIP:g} A for {T_TRIP * 1000:.0f} ms (`I_TRIP_A`, unchanged). "
            f"It sits above twice the walking peak and below the INA226 range ({INA_RANGE:.2f} A on R010), and "
            "at that current two to three DS32xx are stalled: a jam or a fall, not walking.", "",
            "## UBECs", "",
            f"One 6 V {UBEC_A:g} A UBEC per leg; the arms and neck share the left leg's. Sized for a single "
            f"fault: one branch at its limit (a leg at the {I_TRIP:g} A trip, or the arms at their "
            f"{FUSE['arms + neck'][0]:g} A fuse) while the rest walk: worst {one:.1f} A per UBEC. One UBEC for "
            f"everything would see {shared:.1f} A in the same fault, {'inside' if single_ok else 'over'} its "
            f"{UBEC_A:g} A rating. Walking alone needs {a['peak']:.2f} A in all.", "",
            "## Battery", "",
            "| | Battery current |", "| --- | --- |",
            f"| Walking, mean | {i_walk:.2f} A |", f"| Walking, peak | {i_peak:.2f} A |",
            f"| Single fault (one branch at its limit, the rest walking) | {i_worst:.1f} A |",
            f"| Every branch at its limit (the {FUSE['battery'][0]:g} A battery fuse opens) | {i_multi:.1f} A |", "",
            f"In a fault the battery INA226 clips at {INA_RANGE:.2f} A (R010); the fuel gauge rides through it.", "",
            f"Pack {PACK_MAH:.0f} mAh {PACK_C:.0f} C: about **{runtime:.0f} min** of walking at "
            f"{USABLE * 100:.0f} % depth of discharge, logic included ({LOGIC_A:g} A).", "",
            "## Wire (silicone, short runs)", "", "| Branch | Fuse | Smallest wire |", "| --- | --- | --- |"]
    out += [f"| {b} | {FUSE[b][0]:g} A | {gauge[b]} |" for b in FUSE]
    out += ["", "## Checks", "", "| Check | Result | Value |", "| --- | --- | --- |"]
    out += [f"| {n} | {'pass' if ok else 'FAIL'} | {d} |" for n, ok, d in checks]
    (HERE / "power_calc.md").write_text("\n".join(out) + "\n")
    print(f"{len(checks)} checks pass; wrote {(HERE / 'power_calc.md').relative_to(REPO)}")


if __name__ == "__main__":
    main()
