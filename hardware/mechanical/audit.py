"""Mechanical audit of Bazan: the questions check.py, robot.py and shell.py do not ask.

    python audit.py                 # every section, writes out/audit.json (about 15 min)
    python audit.py twin walls      # only the named sections
    python audit.py --strict        # exit 1 if a section reports a defect

Sections
    twin      CAD joint positions and checked ranges against bazan_cad.urdf (the generated digital twin)
    limits    each leg joint driven to its URDF limit: first angle at which the printed parts collide
    reach     how far a leg adducts and an arm rolls inward before it meets the robot's own body
    sweep     every joint alone over the range the CAD was drawn for, every 5 degrees
    matrix    motion test matrix on the whole robot with the shell: overlaps and minimum clearance
    walls     material left around every insert, pilot and clearance hole; thin sections by design
    fits      connector through the cable window, screw engagement, constants copied between files
    print     per part: best print orientation, overhang area, bed contact, height
    mass      printed mass estimate per part, mass and CoM per link, hip-roll holding torque on one foot
    cables    lead length change over the joint ranges against the slack loops
    access    straight tool path to every body and shell screw at the zero pose: what must come off first

Everything here is geometry and arithmetic on the CAD. It does not prove a print succeeds, a wire survives or a
servo holds: docs/engineering/mechanical_verification.md lists the bench tests that do.
"""
import itertools
import json
import sys
from pathlib import Path

import cadquery as cq
import numpy as np

import body as B
import check as C
import hardware as hw
import joint as J
import leg as LEG
import pose as P
import robot as R
import shell as SH

REPO = C.REPO
URDF = REPO / "software" / "ros2" / "src" / "humanoid_description" / "urdf" / "bazan_cad.urdf"   # gen_urdf.py
WIRING_G = 30.0                          # wiring allowance in the pelvis (weigh the harness when it exists)
CLEAR = hw.GAP_MOVING                    # moving parts closer than this are reported as near-collisions
PLA, PETG, TPU = 1.24e-3, 1.27e-3, 1.21e-3   # g/mm3
METAL = {"shin_plate_h": 2.68e-3, "shin_plate_i": 2.68e-3}   # cut plates, solid: Al 5052 (carbon 3K 1.55e-3)
SERVO_OF = dict(hw.JOINT_SERVO)                                   # joint -> servo type
SERVO_G = {n: t.mass for n, t in hw.SERVOS.items()}
LEG_J = list(C.RANGES)


# ------------------------------------------------------------------ the twin
def urdf():
    """{joint: (world position at the zero pose in mm, (lower, upper) in degrees)} from bazan_cad.urdf."""
    import xml.etree.ElementTree as ET
    rows = {}
    for j in ET.parse(URDF).getroot().findall("joint"):
        lim = j.find("limit")
        if j.get("type") != "revolute":
            continue
        xyz = np.array(j.find("origin").get("xyz").split(), float) * 1000
        rows[j.get("name")] = (j.find("parent").get("link"), j.find("child").get("link"), xyz,
                               (np.degrees(float(lim.get("lower"))), np.degrees(float(lim.get("upper")))))
    pos = {"base_link": np.zeros(3)}
    out = {}
    while len(out) < len(rows):
        for name, (parent, child, o, lim) in rows.items():
            if parent in pos and name not in out:
                pos[child] = pos[parent] + o
                out[name] = (pos[child], tuple(round(float(v), 1) for v in lim))
    return out


def twin(leg, body):
    u = urdf()
    rows, bad = [], 0
    cad = {f"l_{n}": (np.array(j.origin) + [0, LEG.HIP_Y, 0], C.RANGES[n], P.AXIS[n]) for n, j in leg.joints.items()}
    for n, j in body.joints.items():
        ax = dict((c[0], c[3]) for c in R.ARM_CHAIN + R.HEAD_CHAIN)[n]
        cad[n if n in ("neck_yaw", "head_pitch") else f"l_{n}"] = (np.array(j.origin), R.RANGES[n], ax)
    for name, (p, rng, ax) in cad.items():
        up, urng = u[name]
        off = (p - up) * (1 - np.abs(ax))             # distance between the two axes, ignoring along-axis position
        covered = rng[0] <= urng[0] + 0.5 and rng[1] >= urng[1] - 0.5
        bad += bool(np.abs(off).max() > 0.5) + (not covered)
        rows.append(dict(joint=name, cad_mm=[round(float(v), 1) for v in p], urdf_mm=[round(float(v), 1) for v in up],
                         axis_offset_mm=[round(float(v), 1) for v in off], cad_range=list(rng), urdf_range=list(urng),
                         range_covered=covered))
    print("twin: joint axes, CAD (robot frame, mm) against bazan_cad.urdf (generated from it: expect no offset)")
    for r in rows:
        print(f"  {r['joint']:18s} CAD {str(r['cad_mm']):24s} URDF {str(r['urdf_mm']):22s} off-axis "
              f"{str(r['axis_offset_mm']):20s} CAD checked {r['cad_range']} URDF {r['urdf_range']}"
              f"{'' if r['range_covered'] else '  RANGE NOT COVERED'}")
    print(f"  hip point above the floor: {-leg.floor:.1f} mm; top of the helmet: {SH.H_TOP - leg.floor:.1f} mm")
    return dict(joints=rows, hip_height=round(-leg.floor, 1), height=round(SH.H_TOP - leg.floor, 1), defects=bad)


# ------------------------------------------------------------------ leg joints to their URDF limits
def leg_hits(leg, q, names, owner):
    comps = P.posed(leg, q, include=("parts", "servos", "horns"))
    out = []
    for a, b in itertools.combinations(names, 2):
        bought = ("servo_", "horn_")
        if a.startswith(bought) and b.startswith(bought) and a.split("_", 1)[1] == b.split("_", 1)[1]:
            continue
        if owner[a] == owner[b] and not (a.startswith("servo_") or b.startswith("servo_")):
            continue
        v = C.overlap(comps[a], comps[b])
        if v > C.TOL:
            out.append((a, b, round(v, 1)))
    return out


def limits(leg):
    names = list(leg.parts) + list(leg.servos()) + list(leg.horns())
    owner = {n: P.owner(leg, n) for n in names}
    u = urdf()
    rows = []
    print("limits: each leg joint alone, from the CAD-checked end to the URDF limit (1° steps)")
    for j in LEG_J:
        for end, cad_end in zip(u[f"l_{j}"][1], C.RANGES[j]):
            sgn = 1.0 if end > 0 else -1.0
            first = None
            for v in np.arange(cad_end, end + sgn * 0.5, sgn) if abs(end) > abs(cad_end) else [end]:
                h = leg_hits(leg, {j: float(v)}, names, owner)
                if h:
                    first = (float(v), h)
                    break
            rows.append(dict(joint=j, urdf_limit=end, cad_checked=cad_end, first_hit=first))
            print(f"  {j:12s} to {end:6.1f}: "
                  + ("clear" if not first else f"COLLIDES from {first[0]:.0f}°: {first[1]}"))
    return dict(rows=rows, defects=sum(r["first_hit"] is not None for r in rows))


def contacts(leg, body, sh, q, mover):
    """Overlapping pairs (a on a link for which mover(link) is true, b on any other link) in pose q, shell on."""
    comps = R.assemble(body, leg, q)
    comps.update(sh.placed(q))
    link = {n: (("head_frame" if n in sh.HEAD else "torso") if n in sh.parts else R.link_of(n, body, leg))
            for n in comps}
    return {(a, b) for a in comps if mover(link[a]) for b in comps
            if link[a] != link[b] and not mover(link[b]) and not R.same_joint_pair(a, b)
            and R.overlap(comps[a], comps[b]) > R.TOL}


def reach(leg, body, sh):
    """Two motions the twin allows that end on the robot's own body: how far each goes before it touches (the other
    side stays at zero). Whole robot with the shell, 1° resolution."""
    out = {}
    for label, key, end, side in (("leg adduction (hip roll toward the other leg)", "l_hip_roll", -25, "L.leg."),
                                  ("arm roll toward the torso", "l_shoulder_roll", -10, "L.")):
        def mover(ln):
            return ln.startswith(side)
        rest = contacts(leg, body, sh, {}, mover)        # what touches by design at the zero pose
        last, hit = 0, None
        for v in range(-1, end - 1, -1):
            hit = sorted(contacts(leg, body, sh, {key: float(v)}, mover) - rest)
            if hit:
                break
            last = v
        out[label] = dict(urdf_limit=end, free_to=last, first_contact=hit[:3] if hit else None)
        print(f"reach: {label}: URDF allows {end}°, free to {last}°" + (f", then {hit[:3]}" if hit else ""))
    return dict(rows=out, defects=sum(r["free_to"] != r["urdf_limit"] for r in out.values()))


def sweep(leg, body, sh, step=5.0):
    """Every joint alone over the range the CAD was drawn for, every `step` degrees: the leg joints on the leg, the
    arm, neck and head joints on the whole robot with the shell (new contacts against the zero pose)."""
    names = list(leg.parts) + list(leg.servos()) + list(leg.horns())
    owner = {n: P.owner(leg, n) for n in names}
    rows = {}
    for j, (lo, hi) in C.RANGES.items():
        vs = np.arange(lo, hi + 0.1, step)
        bad = [float(v) for v in vs if leg_hits(leg, {j: float(v)}, names, owner)]
        rows[j] = dict(range=[lo, hi], poses=len(vs), colliding=bad)
    for j, (lo, hi) in R.RANGES.items():
        head = j in ("neck_yaw", "head_pitch")

        def mover(ln):
            return ln in ("neck_link", "head_frame") if head else ln.startswith("L.") and ".leg." not in ln
        rest = contacts(leg, body, sh, {}, mover)
        vs = np.arange(lo, hi + 0.1, step)
        bad = [float(v) for v in vs if contacts(leg, body, sh, {j if head else f"l_{j}": float(v)}, mover) - rest]
        rows[j] = dict(range=[lo, hi], poses=len(vs), colliding=bad)
    for j, r in rows.items():
        print(f"sweep: {j:15s} {r['range'][0]:5.0f}…{r['range'][1]:4.0f} every {step:.0f}°, {r['poses']:2d} poses: "
              + ("clear" if not r["colliding"] else f"COLLIDES at {r['colliding']}"))
    return dict(rows=rows, defects=sum(bool(r["colliding"]) for r in rows.values()))


# ------------------------------------------------------------------ motion test matrix, whole robot with the shell
def gait_pose(t):
    """Gait targets as robot.assemble wants them: the right leg is the mirrored left, so its roll and yaw flip."""
    import mico_gait as G
    flip = ("r_hip_yaw", "r_hip_roll", "r_ankle_roll", "r_shoulder_roll")
    return {k: float(np.degrees(v)) * (-1 if k in flip else 1) for k, v in G.targets(t).items()}


def matrix_poses():
    import mico_gait as G
    both = lambda **kw: {f"{s}_{k}": v for s in "lr" for k, v in kw.items()}       # noqa: E731
    out = {
        "01 neutral standing": {},
        "02 leg forward, hip pitch -90": {"l_hip_pitch": -90},
        "03 leg back, hip pitch +45": {"l_hip_pitch": 45},
        "04 abduction, hip roll +25 both": both(hip_roll=25),
        "05 adduction, hip roll -25 left": {"l_hip_roll": -25},
        "06 knee 120": {"l_knee": 120},
        "07a ankle pitch +45": {"l_ankle_pitch": 45}, "07b ankle pitch -45": {"l_ankle_pitch": -45},
        "08a ankle roll +30": {"l_ankle_roll": 30}, "08b ankle roll -30": {"l_ankle_roll": -30},
        "11 squat": both(hip_pitch=-50, knee=100, ankle_pitch=-50),
        "11b deep squat, joint limits": both(hip_pitch=-90, knee=120, ankle_pitch=-45),
        "12 weight shift, roll 17": {"l_hip_roll": 17, "r_hip_roll": -17, "l_ankle_roll": -17, "r_ankle_roll": 17},
        "13 arm swing": {"l_shoulder_pitch": -40, "r_shoulder_pitch": 40, "l_elbow": -30, "r_elbow": -30},
        "14 arms raised": dict(both(shoulder_pitch=-180, shoulder_roll=20)),
        "14b arms out, roll 160, elbow -130": both(shoulder_roll=160, elbow=-130),
        "14c arm roll -10 (URDF limit)": both(shoulder_roll=-10),
        "15a head yaw 90 pitch 32": {"neck_yaw": 90, "head_pitch": 32},
        "15b head yaw -90 pitch -30": {"neck_yaw": -90, "head_pitch": -30},
        "16 yaw 45 out, both": both(hip_yaw=45),
        "17 hip pitch 45 + knee 120": {"l_hip_pitch": 45, "l_knee": 120},
        "18 knee 120 + ankle pitch 45": {"l_knee": 120, "l_ankle_pitch": 45},
    }
    for i, t in enumerate(np.linspace(0, G.T_END, 8, endpoint=False)):               # 09 stance and 10 swing phases
        out[f"09/10 gait t={t:.1f}s"] = gait_pose(t)
    return out


def adjacent():
    adj = set()
    for s in "LR":
        chain = ["torso"] + [f"{s}.leg.{c}" for _, _, c in P.CHAIN]
        adj |= set(zip(chain, chain[1:]))
        chain = ["torso"] + [f"{s}.{c[2]}" for c in R.ARM_CHAIN]
        adj |= set(zip(chain, chain[1:]))
    adj |= {("torso", "neck_link"), ("neck_link", "head_frame")}
    return adj | {(b, a) for a, b in adj}


def matrix(leg, body, sh):
    adj = adjacent()
    rows = {}
    for label, q in matrix_poses().items():
        comps = R.assemble(body, leg, q)
        link = {n: R.link_of(n, body, leg) for n in comps}
        for n, s in sh.placed(q).items():
            comps[n] = s
            link[n] = "head_frame" if n in sh.HEAD else "torso"
        skin = set(sh.parts)
        bb = {n: s.BoundingBox() for n, s in comps.items()}
        hits, near = [], []
        for a, b in itertools.combinations(comps, 2):
            if link[a] == link[b] or R.same_joint_pair(a, b):
                continue
            A, Bb = bb[a], bb[b]
            if (A.xmax + CLEAR < Bb.xmin or Bb.xmax + CLEAR < A.xmin or A.ymax + CLEAR < Bb.ymin
                    or Bb.ymax + CLEAR < A.ymin or A.zmax + CLEAR < Bb.zmin or Bb.zmax + CLEAR < A.zmin):
                continue
            v = R.overlap(comps[a], comps[b])
            if v > R.TOL:
                hits.append((a, b, round(v, 1)))
            elif (link[a], link[b]) not in adj or a in skin or b in skin:   # joint neighbours touch by design
                d = comps[a].distance(comps[b])
                if d < CLEAR:
                    near.append((a, b, round(d, 2)))
        rows[label] = dict(q=q, hits=hits, near=near)
        print(f"matrix {label:36s} overlaps {len(hits):2d}  closer than {CLEAR} mm: {len(near):2d}")
        for h in hits:
            print("     OVERLAP", h)
        for h in near:
            print("     near   ", h)
    return dict(rows=rows, defects=sum(bool(r["hits"]) for r in rows.values()))


# ------------------------------------------------------------------ walls around holes
RING = {"ins_m2": 1.6, "ins_m3": 1.6, "ins_m2_thru": 1.6, "ins_m3_thru": 1.6, "boss_m2": 1.2, "pilot_st2": 1.2,
        "clr_m2": 1.2, "clr_m3": 1.2, "clr_so": 1.2, "cb_m2": 1.2, "bearing": 1.2}     # wanted wall, mm


def ring_wall(part, h):
    """Largest of (0.4 .. wanted) mm of material that fully surrounds the hole: over its first 2 mm if blind, over
    0.8 mm past the screw head if through."""
    d = np.asarray(h.dir, float)
    sunk = hw.HOLES[h.kind].cb_h or (2.0 if h.part == "sole" else 2.2 if ".screw" in h.id else 0.0)   # sunk heads
    start = np.asarray(h.pos, float) + d * (sunk + (0.2 if h.part != "sole" else 0.1))
    length = min(h.depth - sunk - 0.4, 2.0) if h.depth else 0.8
    best = 0.0
    for w in (0.4, 0.8, 1.2, 1.6):
        if w > RING[h.kind]:
            break
        ring = cq.Solid.makeCylinder(h.d / 2 + w, length, cq.Vector(*start), cq.Vector(*d)).cut(
            cq.Solid.makeCylinder(h.d / 2 + 0.02, length, cq.Vector(*start), cq.Vector(*d)))
        if ring.intersect(part).Volume() < 0.97 * ring.Volume():
            break
        best = w
    return best


def walls(leg, body):
    thin = []
    n = 0
    for owner in (leg, body):
        for h in owner.holes():
            if h.kind not in RING or h.part not in owner.parts:
                continue
            n += 1
            w = ring_wall(owner.parts[h.part], h)
            if w < RING[h.kind]:
                thin.append(dict(part=h.part, hole=h.id, kind=h.kind, wall=w, wanted=RING[h.kind]))
    print(f"walls: {n} holes checked, {len(thin)} with less material around them than wanted")
    for t in thin:
        print(f"  {t['part']:14s} {t['hole']:24s} {t['kind']:12s} wall {t['wall']:.1f} (wanted {t['wanted']})")
    sections = {
        "servo pocket wall": hw.WALL, "idler wall (M3 insert)": J.IW[1] - J.IW[0],
        "horn recess rim": J.HORN_R - (hw.HORN.disc_d / 2 + 0.2),
        "horn plate web over the horn": J.HP[1] - J.HORN_TOP,
        "horn plate web under the cap recess": J.HP[1] - J.HORN_TOP - J.CAP_T,
        "skin left under a horn pilot hole": 0.4,
        "idler plate lip under the bearing": J.LIP, "side arms": LEG.ARM, "spine": LEG.SPINE,
        "sole plate": LEG.SOLE_PLATE, "shell skin": SH.WALL,
    }
    for k, v in sections.items():
        print(f"  section {k:38s} {v:.2f} mm")
    return dict(checked=n, thin=thin, sections={k: round(v, 2) for k, v in sections.items()}, defects=len(thin))


# ------------------------------------------------------------------ fits that the solids alone do not show
def fits(leg, body, sh):
    out, bad = {}, 0
    win, need = J.WINDOW, (hw.PLUG[0] + 0.5, hw.PLUG[1] + 0.5)
    ok = max(win) >= max(need) and min(win) >= min(need)
    bad += not ok
    out["cable_window"] = dict(window=[round(v, 2) for v in win], plug_needs=[round(v, 2) for v in need], passes=ok)
    print(f"fits: cable window {win[0]:.1f} x {win[1]:.1f}, the 3-pin plug needs {need[0]:.1f} x {need[1]:.1f}: "
          f"{'passes' if ok else 'DOES NOT PASS (servo cannot go in or out with its plug on)'}")
    eng = {
        "servo ear, M2x6 into INS-M2": 6.0 - hw.MG92B.tab_t,
        "leg module, M2x6 through the pelvis counterbore": 6.0 - (B.T0 - hw.HOLES["cb_m2"].cb_h),
        "thrust ring, M2x6 (head sunk 2.2)": 6.0 - (leg.parts["thrust_ring"].BoundingBox().zlen - 2.2),
        "sole, M2x4 (head sunk 2.0 in 3.0 TPU)": 4.0 - (LEG.TPU_SOLE - 2.0),
        "sled, M2x5 through its counterbore": 5.0 - (B.SLED_T - 2.2),
        "flange, M2x6 (shoulder and neck mounts)": 6.0 - B.FLANGE_T,
        f"horn to plate, {hw.horn_screw()} through the horn": int(hw.horn_screw()[-1]) - hw.HORN.disc_t,
    }
    room = {"sole, M2x4 (head sunk 2.0 in 3.0 TPU)": LEG.SOLE_PLATE,
            f"horn to plate, {hw.horn_screw()} through the horn": J.HP[1] - 0.4 - J.HORN_TOP}
    out["engagement"] = {}
    for k, e in eng.items():
        r = room.get(k, 4.0)
        flag = "" if e <= r + 1e-6 else f"  LONGER THAN THE HOLE ({r:.1f}): pokes out {e - r:.1f}"
        flag = flag or ("" if e >= 2.4 else "  SHORT (under 2.4 mm in the thread)")
        if flag and k.startswith("horn to plate") and not hw.HORN.measured:
            flag += ": accepted until the horn is measured (M-2), then hardware.horn_screw() decides"
        else:
            bad += bool(flag)
        out["engagement"][k] = dict(in_thread=round(e, 2), hole=round(r, 2), ok=not flag)
        print(f"  {k:48s} {e:.1f} mm in the thread, hole {r:.1f}{flag}")
    const = {
        "body.Z0 = leg pelvis_floor": (B.Z0, leg.yaw_case_top() - J.IW[0]),
        "pelvis leg screws = hip_mount inserts": (sorted((x, round(y - LEG.HIP_Y, 3)) for x, y in B.Body.HIP_SCREWS
                                                         if y > 0),
                                                  sorted((h.pos[0], h.pos[1]) for h in leg.extra_holes
                                                         if h.id.startswith("pelvis.ins"))),
        "shell bottom tabs = pelvis shell inserts": (sorted(SH.BOT_INSERTS), sorted(
            (h.pos[0], h.pos[1]) for h in body.extra if h.part == "pelvis_plate" and h.id.startswith("shell"))),
        "shell top tabs = deck 2 shell inserts": (sorted(SH.TOP_INSERTS), sorted(
            (h.pos[0], h.pos[1]) for h in body.extra if h.part == "deck_2" and h.id.startswith("shell"))),
        "shell head tabs = head deck inserts": (sorted(SH.HEAD_INSERTS), sorted(
            (h.pos[0], h.pos[1]) for h in body.extra if h.part == "head_frame" and h.id.startswith("shell"))),
    }
    out["constants"] = {}
    for k, (a, b) in const.items():
        same = np.allclose(np.asarray(a, float), np.asarray(b, float), atol=0.01)
        bad += not same
        out["constants"][k] = bool(same)
        print(f"  {k:48s} {'same' if same else 'DIFFER: ' + str(a) + ' / ' + str(b)}")
    out["defects"] = bad
    return out


# ------------------------------------------------------------------ print orientation
DIRS = {"+z up": (0, 0, 1), "-z up": (0, 0, -1), "+x up": (1, 0, 0), "-x up": (-1, 0, 0), "+y up": (0, 1, 0),
        "-y up": (0, -1, 0)}


def orientation(s):
    """For each of the six axis orientations: bed contact cm2, overhang cm2 (faces within 45° of facing down, off
    the bed), height. Returns them sorted best first (least overhang, then most bed contact)."""
    vs, tris = s.tessellate(0.1, 0.3)
    v = np.array([p.toTuple() for p in vs])
    t = np.array(tris)
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    n = np.cross(b - a, c - a)
    area = np.linalg.norm(n, axis=1) / 2
    n = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    cen = (a + b + c) / 3
    if np.sum(np.einsum("ij,ij->i", a, np.cross(b, c))) < 0:
        n = -n
    out = []
    for name, up in DIRS.items():
        up = np.array(up, float)
        h = cen @ up
        h0, h1 = (v @ up).min(), (v @ up).max()
        down = -(n @ up)
        bed = area[(down > 0.99) & (h < h0 + 0.05)].sum()
        over = area[(down > 0.7071) & (h >= h0 + 0.05)].sum()
        out.append((name, round(float(bed) / 100, 1), round(float(over) / 100, 1), round(float(h1 - h0), 1)))
    return sorted(out, key=lambda r: (r[2], -r[1]))


def printing(leg, body, sh):
    rows = {}
    print("print: best orientation per part (bed cm2, overhang cm2, height mm), then the runner-up")
    for group, parts in (("leg", leg.parts), ("body", body.parts), ("shell", sh.parts)):
        for name, s in parts.items():
            o = orientation(s)
            rows[name] = dict(group=group, best=o[0], second=o[1], valid=bool(s.isValid()),
                              solids=len(s.Solids()) if hasattr(s, "Solids") else 1)
            print(f"  {name:15s} {str(o[0]):30s} {str(o[1]):30s} valid {rows[name]['valid']} "
                  f"solids {rows[name]['solids']}")
    bad = sum(not r["valid"] or r["solids"] != 1 for r in rows.values())
    return dict(rows=rows, defects=bad)


# ------------------------------------------------------------------ mass
WITH_SHELL = False   # prototype: bare frame, easy to take apart; shell.py stays for the finished product
RECIPE = {"leg": (1.6, 0.40, PLA), "arm": (1.2, 0.25, PLA), "deck": (1.2, 0.20, PLA), "shell": (9.9, 1.0, PETG)}
ARM_PARTS = ("shoulder_mount", "shoulder_link", "upper_arm", "forearm", "neck_mount", "neck_link", "head_frame",
             "cam_mount")
SIDED = ("shoulder_mount", "shoulder_link", "upper_arm", "forearm", "sled_pca", "sled_ina")


def printed_g(s, recipe):
    """Skin (perimeters and top/bottom of thickness t) solid, the rest at the infill fraction."""
    t, infill, rho = RECIPE[recipe]
    skin = min(s.Volume(), s.Area() * t)
    return rho * (skin + infill * (s.Volume() - skin))


def weigh(leg, body, sh, q=None):
    """Every component of the assembled robot with its link and its mass in grams: the one weighing that audit.py
    mass and gen_urdf.py share. Returns ({printed part: (g, count)}, [(name, link, g, solid or point)])."""
    parts = {}
    for n, s in leg.parts.items():
        parts[f"leg.{n}"] = (TPU * s.Volume() if n == "sole" else METAL[n] * s.Volume() if n in METAL
                             else printed_g(s, "leg"), 2)
    for n, s in body.parts.items():
        parts[n] = (printed_g(s, "arm" if n in ARM_PARTS else "deck"), 2 if n in SIDED else 1)
    for n, s in sh.parts.items():
        parts[f"shell.{n}"] = (printed_g(s, "shell"), 1)
    comps = R.assemble(body, leg, q or {})
    for n, s in (sh.parts.items() if WITH_SHELL else ()):
        comps[n] = s
    items = []
    for n, s in comps.items():
        base = n.split(".")[-1]
        ln = ("head_frame" if n in sh.HEAD else "torso") if n in sh.parts else R.link_of(n, body, leg)
        if base.startswith(("horn_", "cap_", "so4", "so3")):
            continue
        if base.startswith("servo_"):
            g = SERVO_G[SERVO_OF.get(base[6:], "MG92B")] + (3.0 if ".leg." in n else 1.5)   # + hardware allowance
        elif base.startswith("spacer_"):
            g = BRASS * s.Volume()                                          # brass tube on the idler inner race
        elif base == "battery":
            g = B.el.BOARDS["BATT"].mass
        elif base.startswith("board_"):
            g = B.el.BOARDS[base[6:].split("_")[0]].mass
        elif n in sh.parts:
            g = parts[f"shell.{n}"][0]
        elif ".leg." in n:
            g = parts[f"leg.{base}"][0]
        else:
            g = parts[base][0]
        items.append((n, ln, g, s))
    items.append(("wiring", "torso", WIRING_G, np.array([0.0, 0.0, B.Z0T + 10.0])))
    return parts, items


BRASS = 8.5e-3          # g/mm3


def mass(leg, body, sh):
    parts, items = weigh(leg, body, sh)
    frame = sum(g * k for n, (g, k) in parts.items() if not n.startswith("shell.") and n != "leg.sole")
    skin = sum(g * k for n, (g, k) in parts.items() if n.startswith("shell."))
    soles = parts["leg.sole"][0] * 2
    total = sum(i[2] for i in items)
    bought = total - frame - skin - soles
    print("mass: printed parts, estimate (walls + infill recipe per group; weigh the first prints)")
    for n, (g, k) in parts.items():
        print(f"  {n:22s} {g:6.1f} g x {k}")
    print(f"  printed frame {frame:.0f} g, shell {skin:.0f} g, TPU soles {soles:.0f} g; servos, boards, battery, "
          f"spacers and wiring {bought:.0f} g")
    print(f"  whole robot from the CAD: {total:.0f} g")
    # mass and CoM per link at the zero pose, and what the stance hip-roll servo holds on one foot
    link, m = {}, {}
    for n, ln, g, s in items:
        c = s if isinstance(s, np.ndarray) else (lambda v: np.array([v.x, v.y, v.z]))(s.Center())
        link.setdefault(ln, []).append((g, c))
    for ln, items in link.items():
        g = sum(i[0] for i in items)
        m[ln] = (g, sum(i[0] * i[1] for i in items) / g)
    tot = sum(g for g, _ in m.values())
    com = sum(g * c for g, c in m.values()) / tot
    print(f"  per link (zero pose, robot frame): total {tot:.0f} g, CoM {np.round(com, 1)} "
          f"({com[2] - leg.floor:.0f} mm above the floor)")
    for ln, (g, c) in sorted(m.items()):
        print(f"    {ln:22s} {g:6.1f} g  CoM {np.round(c, 1)}")
    # stance on the left foot, pelvis level: everything above the left hip-roll axis hangs on that servo
    above = [k for k in m if not k.startswith("L.leg.") or k == "L.leg.yaw_bracket"]
    tau = 9.81e-6 * sum(m[k][0] * (LEG.HIP_Y - m[k][1][1]) for k in above)            # N.m
    print(f"  hip-roll holding torque, one foot, pelvis level: {tau:.3f} N.m; MG92B stall 0.343, continuous rule 0.114"
          " (the gait leans the pelvis to cut it: docs/engineering/torque_validation.md)")
    return dict(parts={n: [round(g, 1), k] for n, (g, k) in parts.items()}, frame=round(frame), shell=round(skin),
                soles=round(soles), bought=round(bought), total=round(total),
                links={k: [round(g, 1)] + [round(float(v), 1) for v in c] for k, (g, c) in m.items()},
                com=[round(float(v), 1) for v in com], com_above_floor=round(float(com[2] - leg.floor)),
                hip_roll_hold=round(tau, 3), defects=0)


# ------------------------------------------------------------------ cables
def cables(leg):
    """Each lead as a polyline: its exit on the link that holds the servo, then a point behind every joint it
    crosses (fixed to that joint's parent link), then the hip-mount window. Length change against the zero pose."""
    u = urdf()
    routes = leg.cable_routes()
    parent = {jn: p for jn, p, _ in P.CHAIN}

    def length(jn, q):
        T = P.link_locations(leg, q)
        j = leg.joints[jn]
        pts = [(j.holder, j.world((J.CB, 0, J.CASE[0] + 3.5)))]
        pts += [(parent[c], leg.CROSS[c]) for c in routes[jn]["crossed"]] + [("pelvis", leg.window())]
        w = [np.array(cq.Vertex.makeVertex(*map(float, p)).moved(T[ln]).toTuple()) for ln, p in pts]
        return sum(np.linalg.norm(b - a) for a, b in zip(w, w[1:]))

    rows = {}
    print("cables: lead length change over the URDF joint ranges (polyline through the tie points)")
    for jn, r in routes.items():
        base = length(jn, {})
        worst = (0.0, {})
        poses = [{c: float(v)} for c in r["crossed"] for v in u[f"l_{c}"][1]]
        poses += [{c: float(u[f"l_{c}"][1][i]) for c in r["crossed"]} for i in (0, 1)]
        for q in poses:
            d = length(jn, q) - base
            if d > worst[0]:
                worst = (d, q)
        slack = 30.0 * r["loops"]
        rows[jn] = dict(run=r["run"], loops=r["loops"], lead=r["total"], stretch=round(worst[0], 1), at=worst[1],
                        slack=slack)
        print(f"  {jn:12s} run {r['run']} mm, {r['loops']} joints crossed, lead {r['total']} mm; worst stretch "
              f"{worst[0]:.1f} mm of {slack:.0f} mm slack at {worst[1]}")
    return dict(rows=rows, defects=sum(r["stretch"] > r["slack"] for r in rows.values()))


# ------------------------------------------------------------------ tool access to the body and shell screws
def access(leg, body, sh):
    comps = R.assemble(body, leg, {})
    comps.update(sh.parts)
    screws = [(h.part, h.id, np.asarray(h.pos, float), -np.asarray(h.dir, float)) for h in body.extra
              if h.kind in ("cb_m2", "clr_m2", "clr_so") and h.part in body.parts]
    screws += [(p, f"shell{i}", np.asarray(pt, float), -np.asarray(d, float)) for i, (p, pt, d) in enumerate(sh.screws)]
    rows = {}
    for part, hid, pos, out in screws:
        tool = cq.Solid.makeCylinder(1.1, 35.0, cq.Vector(*(pos + 0.5 * out)), cq.Vector(*out))
        own = {part, f"L.{part}", f"R.{part}"}
        blockers = sorted({n for n, s in comps.items() if n not in own and R.overlap(tool, s) > R.TOL})
        rows.setdefault((part, hid.rstrip("0123456789")), set()).update(blockers)
    print("access: 35 mm straight tool over every body and shell screw head, zero pose; what is in the way")
    out = {}
    for (part, grp), blockers in sorted(rows.items()):
        out[f"{part}.{grp}"] = sorted(blockers)
        print(f"  {part:15s} {grp:12s} {'free' if not blockers else 'behind: ' + ', '.join(sorted(blockers))}")
    return dict(rows=out, defects=0)


SECTIONS = ("twin", "limits", "reach", "sweep", "matrix", "walls", "fits", "print", "mass", "cables", "access")


def main():
    want = [a for a in sys.argv[1:] if not a.startswith("--")] or SECTIONS
    leg, body = LEG.Leg(), B.Body()
    sh = None if set(want) <= {"twin", "limits", "walls", "cables"} else SH.BazanShell(body, leg)
    run = {"twin": lambda: twin(leg, body), "limits": lambda: limits(leg), "reach": lambda: reach(leg, body, sh),
           "sweep": lambda: sweep(leg, body, sh),
           "matrix": lambda: matrix(leg, body, sh),
           "walls": lambda: walls(leg, body), "fits": lambda: fits(leg, body, sh),
           "print": lambda: printing(leg, body, sh), "mass": lambda: mass(leg, body, sh),
           "cables": lambda: cables(leg), "access": lambda: access(leg, body, sh)}
    out = {}
    for name in want:
        out[name] = run[name]()
        print(f"-- {name}: {out[name]['defects']} defects\n", flush=True)
    path = Path(__file__).resolve().parent / "out" / "audit.json"
    path.parent.mkdir(exist_ok=True)
    old = json.loads(path.read_text()) if path.exists() else {}
    old.update(out)
    path.write_text(json.dumps(old, indent=1, default=str))
    return 1 if "--strict" in sys.argv and any(v["defects"] for v in out.values()) else 0


if __name__ == "__main__":
    sys.exit(main())
