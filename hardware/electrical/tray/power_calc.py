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
ARM_UBEC_A, ARM_UBEC_BURST_A = 3.0, 5.0            # Hobbywing UBEC 3A (5/6 V jumper at 6 V) for arms + neck (ASSUMED)
I_IDLE = 0.01                      # A per servo holding no load
I_TRIP, T_TRIP = 6.0, 0.200        # software/core/config.h I_TRIP_A, I_TRIP_MS (per leg branch)
INA_RANGE = 0.08192 / 0.010        # INA226 full scale 81.92 mV on the R010 shunt (branch INA226s)
INA_BATT_RANGE = 0.08192 / 0.005   # battery-lead INA226 on R005: sees every single fault unclipped
FUSE = {"left leg": (7.5, 68.0), "right leg": (7.5, 68.0), "arms + neck": (4.0, 17.0), "battery": (15.0, 270.0)}
# (rating A, minimum melting I2t A2s): Littelfuse MINI 297 fast-acting datasheet (5 A 25, 4 A 17, 7.5 A 68, 15 A 270);
# the minimum, so an upstream device that clears below it never melts this fuse
I2T_TYP_FACTOR = 1.2               # typical / minimum melting I2t, 297 series (7.5 A: 82 typ / 68 min)
LOGIC_A = 0.95                     # A from the battery at the logic peak (battery_select.py LOGIC_W)
PACK_MAH, USABLE = 2800.0, 0.85                     # battery_pack.py: 2S1P Molicel P28A, depth of discharge used
PACK = {"cell (Molicel P28A)": 35.0, "BMS (2S 20 A)": 20.0, "XT30PW dock connector": 15.0}   # A continuous
K_PESSIMISTIC = 1.5                # real hobby servos vs the torque-only model (the P0 measurement replaces it)
DERATE = 2.0                       # every part in the power path: rating >= 2 x its pessimistic normal peak
BUCK_A = 3.0                       # 5 V buck, continuous
LOGIC_5V_PEAK_A = 1.23             # battery_select.py logic peak at 5 V
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

    # UBECs: one 8 A per leg, a 3 A one for the arms and neck, so each branch has its own source. Single fault: a leg
    # at the firmware trip; the arms at their UBEC's burst limit (its overload protection drops the arm rail only).
    one = I_TRIP
    check("leg UBEC, single fault (its leg at the trip), inside the continuous rating", one <= UBEC_A,
          f"{one:.1f} A vs {UBEC_A:g} A")
    shared = I_TRIP + stats["right leg"]["peak"] + stats["arms + neck"]["peak"]
    single_ok = shared <= UBEC_A
    check("walking peak, all servos, inside one UBEC", stats["all servos"]["peak"] <= UBEC_A,
          f"{stats['all servos']['peak']:.2f} A")

    # battery
    def batt(servo_a):
        return servo_a * V_RAIL / (UBEC_EFF * V_BAT) + LOGIC_A

    i_walk, i_peak = batt(stats["all servos"]["mean"]), batt(stats["all servos"]["peak"])
    i_worst = batt(max(I_TRIP + stats["right leg"]["peak"] + stats["arms + neck"]["peak"],
                       ARM_UBEC_BURST_A + stats["left leg"]["peak"] + stats["right leg"]["peak"]))   # single fault
    i_multi = batt(2 * I_TRIP + FUSE["arms + neck"][0])                 # everything at its limit: the battery fuse
    runtime = PACK_MAH / 1000 * USABLE / i_walk * 60
    check("battery fuse holds a single fault", i_worst <= FUSE["battery"][0], f"{i_worst:.1f} A")
    check("battery INA226 sees a single fault unclipped", i_worst < INA_BATT_RANGE, f"range {INA_BATT_RANGE:.2f} A")

    # wire: smallest gauge whose rating covers the fuse
    gauge = {b: next(g for g, a in WIRE.items() if a >= FUSE[b][0]) for b in FUSE}

    # derating: every part in the power path carries at most 1 / DERATE of its rating at the pessimistic normal peak
    k = K_PESSIMISTIC
    leg_pk = k * max(stats["left leg"]["peak"], stats["right leg"]["peak"])
    arm_pk = k * stats["arms + neck"]["peak"]
    bat_pk = batt(k * stats["all servos"]["peak"])
    derate = [("UBEC 8 A, left leg", UBEC_A, k * stats["left leg"]["peak"]),
              ("UBEC 8 A, right leg", UBEC_A, k * stats["right leg"]["peak"]),
              ("UBEC 3 A, arms + neck", ARM_UBEC_A, arm_pk),
              ("fuse, leg branch", FUSE["left leg"][0], leg_pk), ("fuse, arms + neck", FUSE["arms + neck"][0], arm_pk),
              ("fuse, battery", FUSE["battery"][0], bat_pk),
              (f"wire, leg ({gauge['left leg']})", WIRE[gauge["left leg"]], leg_pk),
              (f"wire, arms + neck ({gauge['arms + neck']})", WIRE[gauge["arms + neck"]], arm_pk),
              (f"wire, battery ({gauge['battery']})", WIRE[gauge["battery"]], bat_pk),
              ("5 V buck (logic)", BUCK_A, LOGIC_5V_PEAK_A),
              ("INA226 range, leg branch (R010)", INA_RANGE, leg_pk),
              ("INA226 range, battery (R005)", INA_BATT_RANGE, bat_pk)]
    derate += [(n, a, bat_pk) for n, a in PACK.items()]
    for n, rating, load in derate:
        check(f"derating: {n}", rating >= DERATE * load, f"{rating / load:.1f} x ({load:.2f} A of {rating:g} A)")

    # fault carry: a part in series with a fuse carries the fuse rating until it opens
    for n, a in PACK.items():
        check(f"carries the battery fuse: {n}", a >= FUSE["battery"][0], f"{a:g} A vs {FUSE['battery'][0]:g} A")
    for b in FUSE:
        check(f"carries its fuse: wire {b}", WIRE[gauge[b]] >= FUSE[b][0], f"{gauge[b]} {WIRE[gauge[b]]:g} A")

    # selectivity: a fault opens only its own layer. Firmware leg trip < leg fuse < battery fuse; a leg fault lasts
    # at most T_TRIP at the battery, with its UBEC at the burst limit
    i2t_leg_fault_batt = batt(UBEC_BURST_A + stats["right leg"]["peak"] + stats["arms + neck"]["peak"]) ** 2 * T_TRIP
    check("selective: leg fault cleared by firmware before the battery fuse melts",
          i2t_leg_fault_batt < 0.5 * FUSE["battery"][1], f"{i2t_leg_fault_batt:.0f} A²s vs {FUSE['battery'][1]:g} A²s")
    leg_clear = FUSE["left leg"][1] * I2T_TYP_FACTOR * 2.0          # melting + arcing, about twice the melt
    check("selective: leg fuse clears before the battery fuse melts", leg_clear < FUSE["battery"][1],
          f"{leg_clear:.0f} A²s vs {FUSE['battery'][1]:g} A²s")

    # start-up: the firmware enables one joint at a time (soft start), so a UBEC sees the standing load plus one
    # servo slewing at stall current; all at once would be every stall current together
    stand_a = k * 0.35 / 2                                              # battery_select.py standing, per UBEC
    soft = stand_a + max(servo(j).stall_a for j in branches()["left leg"])
    hard = {b: sum(servo(j).stall_a for j in js) for b, js in branches().items()}
    hard_left = hard["left leg"]
    check("derating: start-up with soft start, per UBEC", UBEC_A >= DERATE * soft,
          f"{UBEC_A / soft:.1f} x ({soft:.2f} A)")

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
            f"One 6 V {UBEC_A:g} A UBEC per leg and a 6 V {ARM_UBEC_A:g} A UBEC for the arms and neck: every branch "
            f"has its own source, so a fault on one rail leaves the others up. A leg UBEC sees at most the "
            f"{one:.1f} A firmware trip; sharing a leg UBEC with the arms left it under the 2 x derating. One UBEC for "
            f"everything would see {shared:.1f} A in the same fault, {'inside' if single_ok else 'over'} its "
            f"{UBEC_A:g} A rating. Walking alone needs {a['peak']:.2f} A in all.", "",
            "## Battery", "",
            "| | Battery current |", "| --- | --- |",
            f"| Walking, mean | {i_walk:.2f} A |", f"| Walking, peak | {i_peak:.2f} A |",
            f"| Single fault (one branch at its limit, the rest walking) | {i_worst:.1f} A |",
            f"| Every branch at its limit (the {FUSE['battery'][0]:g} A battery fuse opens) | {i_multi:.1f} A |", "",
            f"The battery INA226 reads up to {INA_BATT_RANGE:.2f} A (R005), every single fault included.", "",
            f"Pack 2S1P {PACK_MAH:.0f} mAh (battery_pack.py): about **{runtime:.0f} min** of walking at "
            f"{USABLE * 100:.0f} % depth of discharge, logic included ({LOGIC_A:g} A), model currents; "
            f"{runtime / K_PESSIMISTIC:.0f} min or more with the servo current x {K_PESSIMISTIC:g}.", "",
            "## Electrical safety: equal margins", "",
            f"Rule 1, derating: every part in the power path carries at most 1/{DERATE:g} of its rating at the walking "
            f"peak x {K_PESSIMISTIC:g} (the model has no servo hunting or friction current).", "",
            "| Part | Rating (A) | Pessimistic peak (A) | Margin |", "| --- | --- | --- | --- |"]
    out += [f"| {n} | {r:g} | {ld:.2f} | {r / ld:.1f} x |" for n, r, ld in sorted(derate, key=lambda d: d[1] / d[2])]
    out += ["", "Rule 2, selectivity: protection opens in layers, smallest and fastest first, so a fault takes out its "
            "own branch only: servo jam → firmware branch trip "
            f"({I_TRIP:g} A, {T_TRIP * 1000:.0f} ms) → branch fuse → UBEC short-circuit limit → battery fuse "
            f"({FUSE['battery'][0]:g} A) → BMS → cell CID. Fuse I²t: Littelfuse MINI 297 minimum melting values.", "",
            "Rule 3, isolation: the 5 V buck for the Radxa and the STM32 (5 V logic → Schottky → Black Pill, "
            "mainboard/architecture.md) takes the battery directly, after the battery "
            "fuse and before any servo branch, so a dead servo rail leaves the controller alive to crouch the "
            "rest.", "",
            f"Start-up: with the soft start (one joint at a time) a UBEC sees {soft:.1f} A; all servos at once would "
            f"draw {hard_left:.1f} A from each leg UBEC (burst rating {UBEC_BURST_A:g} A) and "
            f"{hard['arms + neck']:.1f} A from the arm UBEC ({ARM_UBEC_BURST_A:g} A): over the {I_TRIP:g} A leg trip "
            "and the arm UBEC burst, so the soft start is mandatory.", "",
            "## Wire (silicone, short runs)", "", "| Branch | Fuse | Smallest wire |", "| --- | --- | --- |"]
    out += [f"| {b} | {FUSE[b][0]:g} A | {gauge[b]} |" for b in FUSE]
    out += ["", "## Checks", "", "| Check | Result | Value |", "| --- | --- | --- |"]
    out += [f"| {n} | {'pass' if ok else 'FAIL'} | {d} |" for n, ok, d in checks]
    (HERE / "power_calc.md").write_text("\n".join(out) + "\n")
    print(f"{len(checks)} checks pass; wrote {(HERE / 'power_calc.md').relative_to(REPO)}")


if __name__ == "__main__":
    main()
