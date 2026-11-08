"""Generate everything needed to print and assemble the leg from leg.py:

    mechanical/out/{L,R}_<part>.step and .stl    print files, left leg and its mirror
    docs/mechanical/drawings/<part>.png          3-view drawing, every hole labelled
    docs/mechanical/leg_v3.md                    BOM, hole tables, assembly, servo replacement, clearances

    python gen_docs.py            # writes everything (renders: run render.py first)
    python gen_docs.py --check    # fails if the markdown is stale (CI)
"""
import shutil
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cadquery as cq
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import hardware as hw  # noqa: E402
import joint as J  # noqa: E402
import leg as L  # noqa: E402

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
DOCS = REPO / "docs" / "mechanical"
OUT = HERE / "out"

REF = {  # each part's frame: origin on its reference joint axis, axes as the leg frame
    "yaw_bracket": ("hip_roll", "hip roll axis"), "hip_block": ("hip_pitch", "hip point H (hip pitch axis)"),
    "thigh": ("hip_pitch", "hip point H"), "shin": ("knee", "knee axis"), "ankle_block": ("ankle_pitch", "point A"),
    "foot": ("ankle_roll", "ankle roll axis"), "sole": ("ankle_roll", "ankle roll axis"),
    "hip_mount": ("hip_yaw", "hip yaw axis, mid stack"), "thrust_ring": ("hip_yaw", "hip yaw axis, mid stack"),
}
PRINT = {
    "yaw_bracket": ("PLA", "top plate down on the bed; supports only under the roll horn recess"),
    "hip_block": ("PLA", "idler wall of the hip pitch pocket (−y face) down; no supports inside the pockets"),
    "thigh": ("PLA", "lying on its −y side (idler plate down): both bearing seats print as round holes"),
    "shin": ("PLA", "lying on its −y side: all four plate holes print round"),
    "ankle_block": ("PLA", "as the hip block: −y face down"),
    "foot": ("PLA", "sole plate down; uprights vertical"),
    "sole": ("TPU 95A", "flat, 3 walls, 30 % gyroid"),
    "hip_mount": ("PLA", "top flange down on the bed: the yaw pocket opens upward, no supports"),
    "thrust_ring": ("PLA", "thrust face down on the bed (smooth face against the yaw bracket)"),
}
LEG_HOLE_KINDS = ["ins_m2", "ins_m3", "clr_m2", "clr_m3", "pilot_st2", "bearing", "access"]   # body kinds: body.py
ORDER = ["foot", "sole", "ankle_block", "shin", "thigh", "hip_block", "yaw_bracket", "thrust_ring", "hip_mount"]


def ref_point(leg, part):
    return np.asarray(leg.joints[REF[part][0]].origin, float)


def part_holes(leg):
    out = defaultdict(list)
    for h in leg.holes():
        out[h.part].append(h)
    for p in out:
        out[p].sort(key=lambda h: (h.joint, h.id))
    return out


def fmt(v):
    return "(" + ", ".join(f"{x:+.1f}" for x in v) + ")"


def axis_name(d):
    d = np.round(np.asarray(d, float), 3)
    return {(1, 0, 0): "+x", (-1, 0, 0): "−x", (0, 1, 0): "+y", (0, -1, 0): "−y", (0, 0, 1): "+z",
            (0, 0, -1): "−z"}.get(tuple(d), fmt(d))


# ---------------------------------------------------------------- drawings
def drawing(leg, name, shape, holes, path):
    ref = ref_point(leg, name)
    s = shape.moved(cq.Location(cq.Vector(*(-ref))))
    edges = []
    for e in s.Edges():
        pts = np.array([[p.x, p.y, p.z] for p in e.positions(np.linspace(0, 1, 24))])
        edges.append(pts)
    views = [("Front (from +x)", 1, 2, 0, False), ("Side (from +y)", 0, 2, 1, True), ("Top (from +z)", 0, 1, 2, False)]
    fig, axs = plt.subplots(1, 3, figsize=(16, 6.4), dpi=130)
    for ax, (title, i, j, k, flip) in zip(axs, views):
        for pts in edges:
            u = -pts[:, i] if flip else pts[:, i]
            ax.plot(u, pts[:, j], color="#2A3D5E", lw=0.6)
        for n, h in enumerate(holes):
            p = np.asarray(h.pos) - ref
            u, v = (-p[i] if flip else p[i]), p[j]
            along = abs(h.dir[k]) > 0.9
            if along:
                ax.add_patch(plt.Circle((u, v), h.d / 2, fill=False, color="#E8590C", lw=1.2))
            else:
                d = np.asarray(h.dir)
                du, dv = (-d[i] if flip else d[i]), d[j]
                L_ = h.depth if h.depth else 4
                ax.plot([u, u + du * L_], [v, v + dv * L_], color="#E8590C", lw=2.2)
            ax.annotate(f"H{n + 1}", (u, v), xytext=(4, 4), textcoords="offset points", fontsize=7, color="#C92A2A")
        ax.axhline(0, color="#999", lw=0.5, ls="-.")
        ax.axvline(0, color="#999", lw=0.5, ls="-.")
        ax.set_title(title, fontsize=10)
        ax.set_aspect("equal")
        ax.grid(True, color="#e3e7ee", lw=0.5)
        ax.set_xlabel(("−" if flip else "") + "xyz"[i] + " (mm)")
        ax.set_ylabel("xyz"[j] + " (mm)")
        ax.tick_params(labelsize=7)
    fig.suptitle(f"{name} (left leg) · origin = {REF[name][1]} · holes H1–H{len(holes)}, see the hole table",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


# ---------------------------------------------------------------- markdown
def markdown(leg, holes_by_part):
    s = leg.summary()
    vol = {n: p.Volume() / 1000 for n, p in leg.parts.items()}
    dens = {"PLA": 1.24, "TPU 95A": 1.21}
    lines = []
    a = lines.append
    a("# Mico leg v3: mechanical design and assembly")
    a("")
    a("Generated by `mechanical/gen_docs.py` from `mechanical/leg.py`; do not edit by hand. CI fails if it drifts.")
    a("Left leg shown; the right leg is its mirror in y (`R_*` print files). Dimensions in mm, leg frame: origin at")
    a("the hip point H on the hip-pitch axis, x forward, y outboard, z up, leg straight.")
    a("")
    a("![leg](images/leg_34.jpg)")
    a("")
    a("## Key dimensions (decisions D1 and D2B)")
    a("")
    a("| | Leg v3 | Before (gen_robot.py) |")
    a("| --- | --- | --- |")
    a(f"| Leg width at the joints (caps flush) | {s['stack_width']} | 24–26 |")
    a(f"| Hip roll axis above the hip pitch axis | {s['hip_roll_up']} | 0 (intersecting) |")
    a(f"| Ankle roll axis below the ankle pitch axis | {s['ankle_roll_down']} | 0 (intersecting) |")
    a(f"| Hip → knee → ankle (L1, L2) | {s['L1']}, {s['L2']} | 45, 45 |")
    a(f"| Ankle point A above the floor | {s['A_above_floor']} | 25 |")
    a(f"| Hip point H above the floor | {s['H_above_floor']} | 115 |")
    a(f"| Hip yaw horn plate (underside) above H | {s['yaw_plate_bottom']} | – |")
    a(f"| Pelvis floor (top of `hip_mount`) above H | {s['pelvis_floor']} | 39 (pelvis top) |")
    a(f"| Leg centre lines | y = ±{s['hip_y']} | ±22 |")
    a("")
    a("Where those come from: every joint has the servo shaft on the joint axis, a horn plate on one side and an")
    a("MR63ZZ idler on the other, so the stack across a joint is fixed by the servo and the bearing (table below).")
    a(f"The roll axes sit {s['hip_roll_up']} mm off the pitch axes so the roll servo clears the pitch servo's ears;")
    a(f"the yaw plate and the sole sit where the hip and ankle blocks can swing ±{L.HIP_ROLL_MAX:.0f}° in roll.")
    a("")
    a("## Joint stack (every joint is built the same way)")
    a("")
    a("![exploded](images/leg_exploded.jpg)")
    a("")
    a("Along the shaft, from the case top of the servo (mm):")
    a("")
    a("| Element | From | To |")
    a("| --- | --- | --- |")
    rows = [("Idler plate (driven link), MR63ZZ seat, cap recess", J.IP), ("MR63ZZ bearing", J.BRG),
            ("Holder boss on the inner race", J.BOSS), ("Idler wall (holder) with M3 insert", J.IW),
            ("Servo case", J.CASE), ("Servo ears", J.TAB_A),
            ("Horn disc", (hw.HORN.disc_low, J.HORN_TOP)), ("Horn plate (driven link)", J.HP)]
    for name, (lo, hi) in rows:
        a(f"| {name} | {lo:+.2f} | {hi:+.2f} |")
    a("")
    a("## Bill of materials, one leg")
    a("")
    a("Printed parts:")
    a("")
    a("| Part | Material | Volume (cm³) | Mass, solid (g) | Print orientation | Drawing |")
    a("| --- | --- | --- | --- | --- | --- |")
    tot = 0
    for n in ORDER:
        mat, ori = PRINT[n]
        g = vol[n] * dens[mat]
        tot += g
        a(f"| `{n}` | {mat} | {vol[n]:.1f} | {g:.1f} | {ori} | [drawing](drawings/{n}.png) |")
    a(f"| **Total** | | {sum(vol.values()):.1f} | **{tot:.1f}** | upper bound: printed at 4 walls, 40 % infill"
      " they come out about 30 % lighter | |")
    a("")
    cnt = Counter()
    for h in leg.holes():
        f = h.fastener
        if f.startswith("INS-M2"):
            cnt["INS-M2"] += 1
            cnt["M2x8" if "M2x8" in f else "M2x6"] += 1
        elif f.startswith("INS-M3"):
            cnt["INS-M3"] += 1
        elif f.startswith("MR63ZZ"):
            cnt["MR63ZZ"] += 1
            cnt["W3"] += 1
            cnt["M3x6B"] += 1
        elif f.startswith("ST2x4"):
            cnt["ST2x4"] += 1
    a("Bought parts (servos: 5 × MG92B and the hip-yaw MG90S, which sits in `hip_mount`):")
    a("")
    a("| Code | Qty | Part | Ask the shop for |")
    a("| --- | --- | --- | --- |")
    for code in ("M2x6", "M2x8", "M3x6B", "ST2x4", "INS-M2", "INS-M3", "MR63ZZ", "W3"):
        f = hw.FASTENERS[code]
        a(f"| {code} | {cnt[code]} | {f.desc} | {f.shop} |")
    a("")
    a("Tools: hex keys 1.5 and 2.0 mm, Phillips #0, soldering iron with a heat-set tip (220 °C), 3.0 and 6.0 mm")
    a("drill bits by hand to clean holes, calipers.")
    a("")
    a("## Holes, per part")
    a("")
    a("Positions are the hole entry point in the part frame (origin on the joint axis named, axes as the leg frame).")
    a("Direction is the drilling direction. Depth 0 = through. Every hole is modelled in the print files; the table")
    a("is for checking a print and for pressing the inserts in the right place.")
    a("")
    for n in ORDER:
        hs = holes_by_part.get(n, [])
        if not hs:
            continue
        ref = ref_point(leg, n)
        a(f"### `{n}` · origin: {REF[n][1]}")
        a("")
        a(f"![{n}](drawings/{n}.png)")
        a("")
        a("| # | Hole | Ø | Depth | Position (x, y, z) | Direction | For |")
        a("| --- | --- | --- | --- | --- | --- | --- |")
        for i, h in enumerate(hs):
            p = np.asarray(h.pos) - ref
            dep = f"{h.depth:.1f}" if h.depth else "through"
            a(f"| H{i + 1} | `{h.id}` | {h.d:.2f} | {dep} | {fmt(p)} | {axis_name(h.dir)} | {h.fastener} |")
        a("")
    a("Hole rules: " + "; ".join(f"`{k}` Ø{hw.HOLES[k].d} ({hw.HOLES[k].note})" for k in LEG_HOLE_KINDS) + ".")
    a("")
    a(ASSEMBLY)
    a(MODULE)
    a(SERVICE)
    a("## Cable route")
    a("")
    a("Each lead leaves its pocket through the notch at the far end of the case and runs up the back of the leg,")
    a("crossing every joint just behind its axis (where the joint moves it least), with a slack loop at each crossing,")
    a("through the tie slots (2.5 mm cable ties) in the thigh spine, the shin spine and the yaw bracket's back arm,")
    a("into the pelvis through the window at the back of `hip_mount`. Lengths to the PCA9685 (allowing 30 mm per")
    a("joint crossed and 80 mm inside the pelvis):")
    a("")
    a("| Servo | Run (mm) | Joints crossed | Lead needed (mm) | Stock lead about 250 mm (MEASURE) |")
    a("| --- | --- | --- | --- | --- |")
    for jn, c in leg.cable_routes().items():
        ext = "fits" if c["total"] <= 240 else f"add a {100 if c['total'] <= 340 else 200} mm extension"
        a(f"| {jn} | {c['run']} | {c['loops']} | {c['total']} | {ext} |")
    a("")
    a("Extensions also give each servo its own plug near the pelvis: a servo is unplugged there, not at the PCA9685.")
    a("")
    a("## Clearance and service checks")
    a("")
    a("`python mechanical/check.py` (CI runs `--quick`) proves, on the CAD:")
    a("")
    a("- no two links of a leg overlap with each joint swept over its range (hip yaw ±45°, hip roll ±20°, hip pitch")
    a("  −80…40°, knee 0…120°, ankle pitch ±45°, ankle roll ±20°), over the demo gait and at combined extremes;")
    a("- every servo slides out along its shaft once its driven link is off;")
    a("- every screw can be reached with a straight tool (1.5 / 2.0 mm key, PH0) in the assembled leg, or once the")
    a("  parts listed in the service table are off;")
    a(f"- both legs at y = ±{L.HIP_Y:.0f} never touch with one hip yaw between 15° toe-in and 30° toe-out (the other")
    a("  leg straight), both yaws together up to ±12°, and a weight shift of ±17° roll. Beyond that (both feet yawed")
    a("  the same way more than 12°) the centred feet meet: the gait never does it (it turns one leg, up to 15°), and")
    a("  the joint limits in the controller should forbid it.")
    a("")
    a("![deep](images/leg_deep.jpg)")
    a("")
    a("## Before printing: measure")
    a("")
    a("The servo and horn sizes come from datasheets (`mechanical/hardware.py`, marked MEASURE). Clones differ by a")
    a("few tenths. Measure one servo and one horn with calipers: case length, width, height, ear thickness and height,")
    a("ear hole pitch, shaft offset from the case end, horn disc diameter and thickness, horn height above the case.")
    a("Update `hardware.py`, run `python mechanical/gen_docs.py`, and print a test pocket (`hip_block`) first.")
    a("")
    return "\n".join(lines) + "\n"


MODULE = """## The leg is a module

`hip_mount` carries the hip-yaw servo and bolts to the pelvis floor with 4 × M2×8 from inside the pelvis into
its heat-set inserts. The `thrust_ring` under it carries the robot's weight on the yaw bracket's top face, so the
yaw servo's spline only turns the leg and never holds the robot up. To take a whole leg off: open the pelvis,
unplug the six leads at their extensions, undo the 4 screws. The other leg, the torso and the electronics are not
touched.

![leg module](images/step_10_hip_yaw.jpg)

"""

ASSEMBLY = """## Assembly, one leg

Before anything: power each servo once and drive it to the centre (1500 µs; `mico_link.py ... calibrate` or any
servo tester). Fit the horns only with the servo centred and the leg straight, so the zero pose of the URDF is the
centre of every servo.

1. **Inserts.** Press every heat-set insert (M2 into the servo ear bosses, M3 into the idler walls, M2 into the
   foot plate) with the iron at 220 °C, flush with the surface. Let cool before screwing.
2. **Bearings.** Press an MR63ZZ into every idler plate seat from the outside (thigh ×1, shin ×2, yaw bracket ×1,
   foot ×1). It must spin freely.
3. **Horns to plates.** Open the two horn holes at radius 7 to Ø2.1 so the screws pass the horn. Screw each round
   horn to its horn plate from the servo side with 2 × ST2×4 (heads end up between horn and case).
4. **Servos into pockets.** Slide each servo into its pocket from the shaft side, cable first through the notch at
   the far end, and fix the ears with 2 × M2×6: ankle roll and ankle pitch into `ankle_block`, knee into `thigh`,
   hip roll and hip pitch into `hip_block`, hip yaw (MG90S) into `hip_mount` from below.

   ![pockets](images/step_03_pockets.jpg)
5. **Foot.** Screw the TPU `sole` under `foot` with 4 × M2×6 from below.
6. **Ankle roll.** With the ankle-roll servo centred, push the foot's horn plate onto its spline (foot level with
   the block), drive in the horn centre screw through the access hole, then the idler: W3 washer on the bearing,
   M3×6 button head into the block's M3 insert. Snug, not tight: the bearing must turn.

   ![ankle roll](images/step_05_ankle_roll.jpg)
7. **Ankle pitch.** Same with `shin` on the ankle-pitch servo (shin vertical).

   ![ankle pitch](images/step_06_ankle_pitch.jpg)
8. **Knee.** Same with `shin` on the knee servo in `thigh` (knee straight).

   ![knee](images/step_07_knee.jpg)
9. **Hip pitch.** Same with `thigh` on the hip-pitch servo in `hip_block`.

   ![hip pitch](images/step_08_hip_pitch.jpg)
10. **Hip roll.** Same with `yaw_bracket` on the hip-roll servo (block level).

    ![hip roll](images/step_09_hip_roll.jpg)
11. **Hip yaw.** Push the yaw bracket's top plate onto the hip-yaw servo in `hip_mount` (leg pointing straight
    ahead), horn centre screw from below, then the `thrust_ring` from below with 2 × M2×6 into the mount. The
    bracket must turn freely with 0.2 mm between its top face and the ring; a drop of lithium grease on the face.
12. **Caps.** Press the navy caps into the horn and idler recesses.
13. **Cables.** See the cable route below; tie each lead in the slots, leave the slack loops.
14. **Onto the pelvis.** Set the module under the pelvis floor and drive 4 × M2×8 from inside the pelvis.

"""

SERVICE = """## Replacing one servo

Only the servo's own joint is opened; nothing else in the leg is touched.

| Servo | Lift off | Then |
| --- | --- | --- |
| Ankle roll | `foot` | ears of the servo in `ankle_block` (+x side), slide out forward |
| Ankle pitch | `shin` at the ankle | ears on the +y side of `ankle_block`, slide out outboard |
| Knee | `shin` at the knee | ears on the +y side of `thigh`, slide out outboard |
| Hip pitch | `thigh` | ears on the +y side of `hip_block`, slide out outboard |
| Hip roll | the leg from `yaw_bracket` | ears on the +x side of `hip_block`, slide out forward |
| Hip yaw | the leg from `yaw_bracket`, then `yaw_bracket` (horn screw from below), then `thrust_ring` \
(2 screws) | ears from below in `hip_mount`, slide out downward |

Lifting a link off a joint: pull both navy caps (thumbnail in the notch), undo the horn centre screw through the
access hole (Phillips #0), undo the M3 button head on the idler side (2.0 mm key), pull the link off the spline.
Unplug the servo at its extension lead, undo the two M2 ear screws (1.5 mm key), slide the servo out along its
shaft. Fit the new servo centred, then reverse. About 5 minutes and 6 screws per joint.

"""


def write_print_files(leg):
    OUT.mkdir(exist_ok=True)
    for n, p in leg.parts.items():
        for side, shape in (("L", p), ("R", p.mirror("XZ"))):
            cq.exporters.export(shape, str(OUT / f"{side}_{n}.step"))
            cq.exporters.export(shape, str(OUT / f"{side}_{n}.stl"), tolerance=0.02, angularTolerance=0.1)


def main():
    check = "--check" in sys.argv
    leg = L.Leg()
    hbp = part_holes(leg)
    md = markdown(leg, hbp)
    target = DOCS / "leg_v3.md"
    if check:
        if not target.exists() or target.read_text() != md:
            print("docs/mechanical/leg_v3.md is stale: run python mechanical/gen_docs.py")
            return 1
        print("up to date")
        return 0
    (DOCS / "drawings").mkdir(parents=True, exist_ok=True)
    (DOCS / "images").mkdir(parents=True, exist_ok=True)
    for n in ORDER:
        drawing(leg, n, leg.parts[n], hbp.get(n, []), DOCS / "drawings" / f"{n}.png")
    for f in (HERE / "renders").glob("*.jpg"):
        shutil.copy(f, DOCS / "images" / f.name)
    target.write_text(md)
    write_print_files(leg)
    print("wrote", target)
    return 0


if __name__ == "__main__":
    sys.exit(main())
