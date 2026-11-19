"""Fill the generated tables in docs/engineering/*.md and the README spec row from the sources.

    python docs/engineering/gen_docs.py           # rewrite the blocks between <!-- gen:NAME --> markers
    python docs/engineering/gen_docs.py --check   # exit 1 if any block differs (CI)

Sources (all of them describe the robot generated from the CAD):
    docs/engineering/cad_mass.json                    hardware/mechanical/gen_urdf.py (mass, CoM, height, groups)
    software/ai/sim/mujoco/servo_loads.csv            python sim/mujoco/check_mujoco.py (MuJoCo, the demo gait)
    software/ros2/src/humanoid_control/scripts/bazan_gait.py   gait length, static single-support torques
    hardware/mechanical/hardware.py                   servo types, ratings and masses
    software/ros2/src/humanoid_description/urdf/bazan_cad.urdf (through bazan_gait)

Numbers are rounded so that floating-point noise between machines never changes a document. The stamp is a
hash of the source files, not a commit SHA, so a document stays valid until its inputs actually change.
"""
import csv
import hashlib
import json
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SCRIPTS = REPO / "software" / "ros2" / "src" / "humanoid_control" / "scripts"
CAD_MASS = HERE / "cad_mass.json"
LOADS = REPO / "software" / "ai" / "sim" / "mujoco" / "servo_loads.csv"
HW = REPO / "hardware" / "mechanical" / "hardware.py"
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(HW.parent))
import bazan_gait as gait  # noqa: E402
import hardware  # noqa: E402

G = 9.81
BOM_APLUS = HERE / "bom_aplus.csv"
BASE_BOM_VND = 5_785_000      # total of design/sketch_shell_a/BOM.dc.html v0.8 (robot, before A+)
SOURCES = [CAD_MASS, gait.URDF, LOADS, SCRIPTS / "bazan_gait.py", HW, BOM_APLUS]
LEG = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")
PEAK_FLAG, RMS_FLAG, TIME_FLAG = 0.80, 0.60, 0.10


def kgcm(nm):
    return nm * 100 / G


def f1(v):
    """One decimal, without a negative zero."""
    return f"{v + 0.0:.1f}".replace("-0.0", "0.0")


def table(head, rows, align=None):
    align = align or ["---"] * len(head)
    out = ["| " + " | ".join(head) + " |", "| " + " | ".join(align) + " |"]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def stamp():
    h = hashlib.sha256()
    for f in SOURCES:
        h.update(f.read_bytes().replace(b"\r\n", b"\n"))
    names = ", ".join(f"`{f.relative_to(REPO).as_posix()}`" for f in SOURCES)
    return f"Generated from {names} · source hash `{h.hexdigest()[:12]}`"


# ---------------------------------------------------------------- data
def load():
    r = json.loads(CAD_MASS.read_text())
    with open(LOADS, encoding="utf-8") as f:
        mj = {row["joint"]: row for row in csv.DictReader(f)}
    return r, mj


def servo_of(joint):
    return hardware.SERVOS[hardware.JOINT_SERVO[joint[2:] if joint[:2] in ("l_", "r_") else joint]]


def stall_nm(joint):
    return servo_of(joint).stall_kgcm * G / 100


def static_cases():
    """Hold torque (N.m) per joint in the gait's start pose and halfway through a swing on the left foot."""
    cases = {}
    stand = gait.crouch_pose(1.0, sag=False)[0]
    cases["stand"] = gait.MODEL.hold_torques(gait.MODEL.fk(stand), np.eye(3), {"l": 0.5, "r": 0.5})
    single = gait.single_support("l")
    T = gait.MODEL.fk(single)
    cases["single"] = gait.MODEL.hold_torques(T, T["l_sole"][0].T, {"l": 1.0})    # the stance sole flat
    return cases


# ---------------------------------------------------------------- mass_budget.md
def mass_blocks(r):
    inertia = r["inertia_about_com_gmm2"]
    summary = table(
        ["Quantity", "Value"],
        [["Total", f"**{r['total_g']:.0f} g**"],
         ["Height (floor to the top of the head)", f"{r['height_mm']:.0f} mm"],
         ["CoM above the floor (straight legs)", f"{r['com_above_floor_mm']:.0f} mm"],
         ["CoM x / y from the middle of the hip points", f"{f1(r['com_mm'][0])} / {f1(r['com_mm'][1])} mm"],
         ["Whole-body inertia about CoM (roll, pitch, yaw)",
          " / ".join(f"{v / 1e6:.2f}" for v in inertia) + " ×10⁻³ kg·m²"],
         ["LIPM time constant √(z/g)", f"{r['lipm_tc_ms']:.0f} ms"]],
        ["---", "---:"])
    groups = table(["Group", "g", "Share"],
                   [[k, f"{v:.1f}", f"{100 * v / r['total_g']:.0f}%"] for k, v in r["groups_g"].items()],
                   ["---", "---:", "---:"])
    order = ["base_link", "neck_yaw_link", "head_pitch_link"] + \
        [f"l_{j}_link" for j in ("shoulder_pitch", "shoulder_roll", "elbow") + LEG]
    rows = [[f"`{k}`", f"{r['links'][k]['g']:.1f}", " / ".join(f1(c) for c in r["links"][k]["com_mm"]),
             " / ".join(f"{v:,.0f}" for v in r["links"][k]["I_diag_gmm2"])] for k in order]
    links = table(["Link", "g", "CoM x / y / z (mm)", "Ixx / Iyy / Izz (g·mm²)"], rows, ["---", "---:", "---:", "---:"])
    return dict(summary=summary, groups=groups, links=links)


# ---------------------------------------------------------------- torque_validation.md
def torque_blocks(mj):
    cases = static_cases()
    rows, flagged, others = [], [], []
    for name in gait.JOINTS:
        stall = stall_nm(name)
        cont = stall / 3
        g = mj[name]
        p99, rms = float(g["p99_Nm"]), float(g["rms_Nm"])
        above = float(g["s_above_continuous"]) / (gait.T_END + 3.0)        # check_mujoco runs 2 s + demo + 1 s
        mirror = ("r_" + name[2:]) if name.startswith("l_") else ("l_" + name[2:]) if name.startswith("r_") else name
        static = max(abs(c.get(n, 0.0)) for c in cases.values() for n in (name, mirror))
        flags = []
        if p99 / stall > PEAK_FLAG:
            flags.append("peak")
        if rms / cont > RMS_FLAG or above > TIME_FLAG:
            flags.append("thermal")
        row = [f"`{name}`", hardware.JOINT_SERVO[name[2:] if name[:2] in ("l_", "r_") else name],
               f"{kgcm(stall):.2f}", f"{kgcm(p99):.2f} ({100 * p99 / stall:.0f}%)",
               f"{kgcm(rms):.2f} ({100 * rms / cont:.0f}%)", f"{100 * above:.0f}%",
               f"{kgcm(static):.2f} ({100 * static / cont:.0f}%)", ", ".join(flags) or "–"]
        if name.split("_", 1)[1] in LEG:
            rows.append(row)
            if flags:
                flagged.append((name, flags, p99 / stall, rms / cont, above))
        else:
            others.append(row)
    head = ["Joint", "Servo", "Stall kg·cm", "Peak p99 (/ stall)", "RMS (/ continuous)", "Time above continuous",
            "Static worst (/ continuous)", "Flag"]
    al = ["---", "---", "---:", "---:", "---:", "---:", "---:", "---"]
    note = (f"MuJoCo on the CAD robot ({gait.MODEL.total * 1000:.0f} g), the demo gait ({gait.T_END:.1f} s). "
            "Static worst is the larger of the left and right joint over two planned poses: the gait's start pose "
            "and halfway through a swing on one foot.")
    legs = note + "\n\n" + table(head, rows, al)
    if flagged:
        lines = [f"- `{n}`: {', '.join(f)} (peak {100 * p:.0f}% of stall, RMS {100 * q:.0f}% of continuous, "
                 f"{100 * a:.0f}% of the run above continuous)" for n, f, p, q, a in flagged]
        worst_peak = max(p for _, _, p, _, _ in flagged)
        verdict = ("No leg servo is shown to fail. " if worst_peak <= 1 else "") + \
            "These joints need bench data before P1:\n\n" + "\n".join(lines)
    else:
        verdict = "No leg joint is flagged at the current mass."
    return dict(legs=legs, verdict=verdict, others=table(head, others, al))


# ---------------------------------------------------------------- hardware_architecture.md
def hardware_blocks(r):
    alloc = {}
    for j in gait.JOINTS:
        alloc.setdefault(hardware.JOINT_SERVO[j[2:] if j[:2] in ("l_", "r_") else j], []).append(j)
    rows = []
    for name, js in alloc.items():
        sv = hardware.SERVOS[name]
        names = sorted({j.split("_", 1)[1] if j[:2] in ("l_", "r_") else j for j in js})
        rows.append([name, len(js), f"{sv.stall_kgcm:.2f}", f"{sv.mass:g}", f"{len(js) * sv.mass:.1f}",
                     ", ".join(names)])
    total = sum(x[1] for x in rows)
    mass = sum(float(x[4]) for x in rows)
    rows.append(["**Total**", total, "", "", f"**{mass:.1f}**", f"{100 * mass / r['total_g']:.0f}% of "
                 f"{r['total_g']:.0f} g"])
    return dict(servos=table(["Servo", "Count", "Stall kg·cm", "g each", "g total", "Joints"], rows,
                             ["---", "---:", "---:", "---:", "---:", "---"]))


def vnd(v):
    return f"{v:,.0f}"


def load_bom():
    with open(BOM_APLUS, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return rows, sum(int(r["qty"]) * int(r["unit_vnd"]) for r in rows)


def bom_blocks():
    rows, total = load_bom()
    body = []
    for r in rows:
        shop = f"[{r['shop']}]({r['url']})" if r["url"] else r["shop"]
        price = vnd(int(r["unit_vnd"])) + (" (est.)" if r["status"] == "estimate" else "")
        body.append([r["part"], r["qty"], price, vnd(int(r["qty"]) * int(r["unit_vnd"])), shop, r["note"]])
    body.append(["**A+ total**", "", "", f"**{vnd(total)}**", "", ""])
    body.append(["**Robot total** (BOM + A+)", "", "", f"**{vnd(BASE_BOM_VND + total)}**", "", ""])
    dates = sorted({r["checked"] for r in rows})
    return dict(bom=table(["Part", "Qty", "VND each", "VND", "Shop", "Note"], body,
                          ["---", "---:", "---:", "---:", "---", "---"]) + f"\n\nPrices checked {', '.join(dates)}.")


# ---------------------------------------------------------------- README.md
def readme_blocks(r):
    spec = table(["Height", "Mass", "DoF", "Compute", "Sensors", "BOM"],
                 [[f"{r['height_mm']:.0f} mm", f"{r['total_g']:.0f} g",
                   f"{len(gait.JOINTS)} (legs 2×6, arms 2×3, neck 2)",
                   "Radxa Zero 3W (Cortex-A55 + NPU) · STM32F411 (Cortex-M4)",
                   "IMU, camera, ToF, 2 mics, 4 foot FSRs",
                   f"≈ {(BASE_BOM_VND + load_bom()[1]) / 1e6:.2f}M VND"]])
    return dict(readme_spec=spec)


# ---------------------------------------------------------------- fill
def fill(text, blocks):
    for name, body in blocks.items():
        pat = re.compile(rf"(<!-- gen:{name} -->\n).*?(<!-- /gen:{name} -->)", re.S)
        text, n = pat.subn(lambda m: m.group(1) + body + "\n" + m.group(2), text)
        if n != 1:
            raise SystemExit(f"marker gen:{name} found {n} times")
    return text


def main():
    check = "--check" in sys.argv
    r, mj = load()
    st = stamp()
    docs = {
        "mass_budget.md": mass_blocks(r),
        "torque_validation.md": torque_blocks(mj),
        "hardware_architecture.md": dict(hardware_blocks(r), **bom_blocks()),
        "README.md": readme_blocks(r),
    }
    stale = []
    for fname, blocks in docs.items():
        p = HERE / fname
        if fname == "README.md":
            p = REPO / fname
        old = p.read_text(encoding="utf-8")
        new = fill(old, blocks if fname == "README.md" else dict(blocks, stamp=st))
        if new != old:
            stale.append(fname)
            if not check:
                p.write_text(new, encoding="utf-8")
    if check and stale:
        print("out of date, run python docs/engineering/gen_docs.py:", ", ".join(stale))
        sys.exit(1)
    print("up to date" if check else f"wrote {len(stale)} of {len(docs)} documents")


if __name__ == "__main__":
    main()
