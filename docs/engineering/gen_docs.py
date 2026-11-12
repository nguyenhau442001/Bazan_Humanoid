"""Fill the generated tables in docs/engineering/*.md and the README spec row from the sources. Stdlib only.

    python docs/engineering/gen_docs.py           # rewrite the blocks between <!-- gen:NAME --> markers
    python docs/engineering/gen_docs.py --check   # exit 1 if any block differs (CI)

Sources:
    docs/design/blender/mass_report.json     python docs/design/blender/mico_mass.py
    sim/mini_humanoid/tables/mj_servo_loads.csv   python sim/mujoco/check_mujoco.py (MuJoCo)
    sim/mini_humanoid/gen_robot.py           servo types and joint allocation
    src/humanoid_control/scripts/mico_gait.py   gait duration

Numbers are rounded so that floating-point noise between machines never changes a document. The stamp is a
hash of the source files, not a commit SHA, so a document stays valid until its inputs actually change.
"""
import csv
import hashlib
import importlib.util
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SIM = REPO / "sim" / "mini_humanoid"
BLENDER = REPO / "design" / "blender"
GAIT = REPO / "src" / "humanoid_control" / "scripts" / "mico_gait.py"
sys.path.insert(0, str(SIM))
import gen_robot  # noqa: E402

G = 9.81
BOM_APLUS = HERE / "bom_aplus.csv"
BASE_BOM_VND = 5_650_000      # total of docs/design/sketch_shell_a/BOM.dc.html (P3 robot, before A+)
SOURCES = [BLENDER / "mico_parts.py", BLENDER / "mico_mass.py", SIM / "gen_robot.py",
           SIM / "tables" / "mj_servo_loads.csv", GAIT, BOM_APLUS]
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
    r = json.loads((BLENDER / "mass_report.json").read_text())
    with open(SIM / "tables" / "mj_servo_loads.csv", encoding="utf-8") as f:
        mj = {row["joint"]: row for row in csv.DictReader(f)}
    spec = importlib.util.spec_from_file_location("mico_gait", GAIT)
    gait = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gait)
    sim_mass = sum(g.mass for b in gen_robot.walk(gen_robot.ROBOT) for g in b.geoms)
    return r, mj, gait.T_END, sim_mass


# ---------------------------------------------------------------- mass_budget.md
def mass_blocks(r, sim_mass):
    lw = r["variants"]["LW-PLA, walls as drawn"]
    lipm = r["lipm"]
    summary = table(
        ["Quantity", "Value"],
        [["Total, PLA shell (baseline)", f"**{r['total_g']:.0f} g**"],
         ["Total, LW-PLA shell", f"{lw:.0f} g"],
         ["Height (top of head shell)", f"{r['height_mm']:.0f} mm"],
         ["CoM above the floor", f"{r['com_mm'][2]:.0f} mm"],
         ["CoM x / y from the hip point", f"{f1(r['com_mm'][0])} / {f1(r['com_mm'][1])} mm"],
         ["Whole-body inertia about CoM (roll, pitch, yaw)",
          " / ".join(f"{v / 1e6:.2f}" for v in r["I_body_principal"]) + " ×10⁻³ kg·m²"],
         ["LIPM time constant √(z/g)", f"{lipm['tc_s'] * 1000:.0f} ms"],
         ["Simulation model `mass:=sim` (for comparison)", f"{sim_mass:.0f} g"]],
        ["---", "---:"])
    groups = table(["Group", "g", "Share"],
                   [[k, f"{v:.1f}", f"{100 * v / r['total_g']:.0f}%"] for k, v in r["categories"].items()],
                   ["---", "---:", "---:"])
    variants = table(["Shell material and wall", "Total g", "vs baseline"],
                     [[k, f"{v:.0f}", f"{v - r['total_g']:+.0f}" if abs(v - r['total_g']) >= 0.5 else "–"]
                      for k, v in r["variants"].items()],
                     ["---", "---:", "---:"])
    order = ["base", "neck_yaw", "head_pitch"] + [f"l_{j}" for j in ("shoulder_pitch", "shoulder_roll", "elbow")] \
        + [f"l_{j}" for j in LEG]
    rows = []
    for k in order:
        v = r["links"][k]
        i = v["I"]
        rows.append([f"`{k}_link`", f"{v['mass']:.1f}", " / ".join(f1(c) for c in v["com"]),
                     " / ".join(f"{i[n][n]:,.0f}" for n in range(3))])
    links = table(["Link", "g", "CoM x / y / z (mm)", "Ixx / Iyy / Izz (g·mm²)"], rows, ["---", "---:", "---:", "---:"])
    return dict(summary=summary, groups=groups, variants=variants, links=links)


# ---------------------------------------------------------------- torque_validation.md
def torque_blocks(r, mj, t_end, sim_mass):
    scale = r["total_g"] / sim_mass
    joints = {j["joint"]: j for j in r["joints"]}
    rows, flagged, others = [], [], []
    for name, j in joints.items():
        stall = gen_robot.SERVOS[j["servo"]]["stall"]
        cont = stall / 3
        g = mj.get(name)
        p99 = float(g["p99_Nm"]) * scale if g else 0.0
        rms = float(g["rms_Nm"]) * scale if g else 0.0
        above = float(g["s_above_continuous"]) / t_end if g else 0.0
        mirror = joints.get(("r_" + name[2:]) if name.startswith("l_") else ("l_" + name[2:]), j)
        static = max(j["new"], mirror["new"]) * G / 100     # kg.cm -> N.m; static cases load one side only
        flags = []
        if p99 / stall > PEAK_FLAG:
            flags.append("peak")
        if rms / cont > RMS_FLAG or above > TIME_FLAG:
            flags.append("thermal")
        row = [f"`{name}`", j["servo"], f"{kgcm(stall):.2f}", f"{kgcm(p99):.2f} ({100 * p99 / stall:.0f}%)",
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
    note = (f"Mass ratio used for the MuJoCo columns: {r['total_g']:.0f} g / {sim_mass:.0f} g = {scale:.3f}. "
            f"Gait run: {t_end:.1f} s. Static worst is the larger of the left and right joint, since the static "
            "cases stand on the left foot only.")
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
    for b in gen_robot.walk(gen_robot.ROBOT):
        if b.joint:
            alloc.setdefault(b.joint.servo, []).append(b.joint.name)
    rows = []
    for servo, js in alloc.items():
        s = gen_robot.SERVOS[servo]
        names = sorted({j.split("_", 1)[1] if j[:2] in ("l_", "r_") else j for j in js})
        rows.append([servo, len(js), f"{kgcm(s['stall']):.2f}", f"{s['mass']:g}", f"{len(js) * s['mass']:.1f}",
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
    lw = r["variants"]["LW-PLA, walls as drawn"]
    n = sum(1 for b in gen_robot.walk(gen_robot.ROBOT) if b.joint)
    spec = table(["Height", "Mass", "DoF", "Compute", "Sensors", "BOM"],
                 [[f"{r['height_mm']:.0f} mm", f"{r['total_g']:.0f} g ({lw:.0f} g in LW-PLA)",
                   f"{n} (legs 2×6, arms 2×3, neck 2)",
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
    r, mj, t_end, sim_mass = load()
    st = stamp()
    docs = {
        "mass_budget.md": mass_blocks(r, sim_mass),
        "torque_validation.md": torque_blocks(r, mj, t_end, sim_mass),
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
