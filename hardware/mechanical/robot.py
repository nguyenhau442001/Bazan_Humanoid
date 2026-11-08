"""The whole of Mico: both legs (leg.py) on the body (body.py), posed, and the whole-body checks.

    python robot.py            # static fit, arm/neck/head sweeps, tool access, servo removal; exit 1 on a problem
    python robot.py --quick    # fewer poses

Right side = mirror of the left in y. Arm and head angles follow the URDF signs (shoulder pitch about +y, roll and
elbow as named in mico.urdf.xacro). A pose is a dict like {"l_shoulder_pitch": -90, "neck_yaw": 30}.
"""
import itertools
import math
import sys

import cadquery as cq
import numpy as np
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf

import body as B
import leg as LEG
import pose as P

TOL = 0.05   # mm³; parts that touch by design (servo ears on bosses, horn in its recess)

ARM_CHAIN = [("shoulder_pitch", "torso", "shoulder_link", (0, 1, 0)),
             ("shoulder_roll", "shoulder_link", "upper_arm", (1, 0, 0)),
             ("elbow", "upper_arm", "forearm", (0, 1, 0))]
HEAD_CHAIN = [("neck_yaw", "torso", "neck_link", (0, 0, 1)), ("head_pitch", "neck_link", "head_frame", (0, 1, 0))]
RANGES = {"shoulder_pitch": (-180, 60), "shoulder_roll": (0, 160), "elbow": (-130, 0), "neck_yaw": (-90, 90),
          "head_pitch": (-30, 32)}
TORSO = ["pelvis_plate", "deck_1", "deck_2", "sled_pca", "sled_perf", "sled_radxa", "sled_ubec", "sled_buck",
         "sled_ina", "shoulder_mount", "neck_mount"]


def mirror_y(s):
    return s.mirror("XZ")


def chain_T(body, chain, q, mirror=False):
    T = {"torso": gp_Trsf()}
    for jn, parent, child, ax in chain:
        j = body.joints[jn]
        o = np.array(j.origin, float)
        axis = np.array(ax, float)
        ang = q.get(jn, 0.0)
        if mirror:
            o[1] = -o[1]
            # mirrored chain: rotations about x and z flip sign, about y stay (roll -10..160 left = 10..-160 right)
            ang = ang if ax == (0, 1, 0) else -ang
        r = gp_Trsf()
        r.SetRotation(gp_Ax1(gp_Pnt(*map(float, o)), gp_Dir(*map(float, axis))), math.radians(ang))
        T[child] = T[parent].Multiplied(r)
    return {k: cq.Location(v) for k, v in T.items()}


def owner(body, name):
    if name in body.parts:
        return "torso" if name in TORSO else name
    kind, jn = name.split("_", 1)
    j = body.joints[jn]
    link = j.holder if kind == "servo" else j.driven
    return "torso" if link in TORSO else link


def assemble(body, leg, q=None, include_legs=True):
    """{name: solid} for the whole robot in pose q. Names carry L./R. for sided parts."""
    q = q or {}
    out = {}
    side_q = {"L": {k[2:]: v for k, v in q.items() if k.startswith("l_")},
              "R": {k[2:]: v for k, v in q.items() if k.startswith("r_")}}
    head_q = {k: v for k, v in q.items() if k in ("neck_yaw", "head_pitch")}
    comps = dict(body.parts)
    comps.update(body.servos())
    comps.update(body.horns())
    arm_names = [n for n in comps if owner(body, n) in ("shoulder_link", "upper_arm", "forearm")
                 or n in ("servo_shoulder_pitch",)]
    head_names = [n for n in comps if owner(body, n) in ("neck_link", "head_frame") or n == "cam_mount"
                  or n == "servo_neck_yaw"]
    for n, s in comps.items():
        if n in arm_names or n in ("shoulder_mount", "sled_pca", "sled_ina"):
            continue
        if n in head_names:
            continue
        out[n] = s
    # sided torso parts (left built, right mirrored)
    for n in ("shoulder_mount", "sled_pca", "sled_ina"):
        out[f"L.{n}"] = comps[n]
        out[f"R.{n}"] = mirror_y(comps[n])
    for side, mir in (("L", False), ("R", True)):
        locs = chain_T(body, ARM_CHAIN, side_q[side], mir)
        for n in arm_names:
            s = mirror_y(comps[n]) if mir else comps[n]
            out[f"{side}.{n}"] = s.moved(locs[owner(body, n) if n != "servo_shoulder_pitch" else "torso"])
    locs = chain_T(body, HEAD_CHAIN, head_q)
    for n in head_names:
        o = "head_frame" if n == "cam_mount" else ("torso" if n == "servo_neck_yaw" else owner(body, n))
        out[n] = comps[n].moved(locs[o])
    for i, s in body.standoffs().items():
        out[i] = s
    out["battery"] = body.battery()
    for k, s in body.boards.items():
        out[f"board_{k}"] = s.moved(locs["head_frame"]) if k == "DISP" else s
    if include_legs:
        for side, sy in (("L", 1), ("R", -1)):
            lq = {k: v for k, v in side_q[side].items() if k in LEG.Leg().joints} if False else side_q[side]
            posed = P.posed(leg, {k: v for k, v in lq.items() if k in P.AXIS})
            for n, s in posed.items():
                s2 = s if sy > 0 else s.mirror("XZ")
                out[f"{side}.leg.{n}"] = s2.moved(cq.Location(cq.Vector(0, sy * LEG.HIP_Y, 0)))
    return out


def link_of(name, body, leg):
    """Rigid body a component belongs to, for deciding which overlaps matter."""
    side = name.split(".")[0] if name[:2] in ("L.", "R.") else ""
    base = name.split(".")[-1]
    if ".leg." in name:
        o = P.owner(leg, base)
        return f"{side}.leg.{o}" if o != "pelvis" else "torso"
    if base == "board_DISP":
        return "head_frame"
    if base.startswith(("so40", "so35", "board_")) or base == "battery":
        return "torso"
    if base in body.parts or base.startswith(("servo_", "horn_")):
        o = owner(body, base)
        if base == "cam_mount":
            o = "head_frame"
        return f"{side}.{o}" if side and o != "torso" else o
    return base


def overlap(a, b):
    ba, bb = a.BoundingBox(), b.BoundingBox()
    if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin or ba.zmax < bb.zmin
            or bb.zmax < ba.zmin):
        return 0.0
    return a.intersect(b).Volume()


def same_joint_pair(a, b):
    """A servo and its own horn, or a servo and the link that holds it, touch by design (checked in joint.py)."""
    ka, kb = a.split(".")[-1], b.split(".")[-1]
    for x, y in ((ka, kb), (kb, ka)):
        if x.startswith("servo_") and y.startswith("horn_") and x[6:] == y[5:]:
            return True
    return False


def collisions(body, leg, q, names=None, against=None):
    comps = assemble(body, leg, q)
    names = names or list(comps)
    links = {n: link_of(n, body, leg) for n in comps}
    hits = []
    pairs = itertools.combinations(names, 2) if against is None else \
        ((a, b) for a in names for b in against if a != b)
    for a, b in pairs:
        if same_joint_pair(a, b):
            continue
        servo = a.split(".")[-1].startswith("servo_") or b.split(".")[-1].startswith("servo_")
        if links[a] == links[b] and not servo:
            continue
        v = overlap(comps[a], comps[b])
        if v > TOL:
            hits.append((a, b, round(v, 2)))
    return hits


def sweep_poses(quick):
    k = 4 if quick else 7
    out = []
    for j, (lo, hi) in RANGES.items():
        for v in np.linspace(lo, hi, k):
            if j in ("neck_yaw", "head_pitch"):
                out.append({j: float(v)})
            else:
                out.append({f"l_{j}": float(v), f"r_{j}": float(v)})
    out += [{"l_shoulder_pitch": -90, "l_elbow": -130, "r_shoulder_pitch": -90, "r_elbow": -130},
            {"l_shoulder_roll": 160, "l_elbow": -130, "r_shoulder_roll": 160, "r_elbow": -130},
            {"l_shoulder_pitch": -180, "l_shoulder_roll": 90, "neck_yaw": 90, "head_pitch": 32},
            {"neck_yaw": -90, "head_pitch": -30, "l_shoulder_pitch": 60, "r_shoulder_pitch": 60},
            {"l_hip_pitch": -80, "l_knee": 120, "l_shoulder_pitch": 60, "l_elbow": -130}]
    return out


SERVICE_POSE = {"elbow": {"l_shoulder_roll": 90, "r_shoulder_roll": 90}}   # arm out sideways to reach the elbow


def tool_access(body, leg):
    out = []
    for j in body.joints.values():
        q = SERVICE_POSE.get(j.name, {})
        comps = assemble(body, leg, q)
        locs = {"L": chain_T(body, ARM_CHAIN, {k[2:]: v for k, v in q.items() if k.startswith("l_")}),
                "R": chain_T(body, ARM_CHAIN, {k[2:]: v for k, v in q.items() if k.startswith("r_")}, True)}
        for label, tool, off in j.tool_paths():
            if j.name == "neck_yaw" and "horn" in label:
                off = "head_frame+servo_head_pitch"
            for side in (("L", "R") if j.name in ("shoulder_pitch", "shoulder_roll", "elbow") else ("",)):
                t = tool.mirror("XZ") if side == "R" else tool
                if j.name == "elbow":
                    t = t.moved(locs[side]["upper_arm"])
                elif j.name == "shoulder_roll":
                    t = t.moved(locs[side]["shoulder_link"])
                gone = set()
                for o in (off.split("+") if off else []):
                    chain = ["shoulder_link", "upper_arm", "forearm"] if j.name in (
                        "shoulder_pitch", "shoulder_roll", "elbow") else ["neck_link", "head_frame"]
                    gone |= set(chain[chain.index(o):]) if o in chain else {o}
                blockers = []
                for n, s in comps.items():
                    ln = link_of(n, body, leg)
                    base = ln.split(".")[-1]
                    if (base in gone or n.split(".")[-1] in gone) and (not side or ln.startswith(side) or ln == base):
                        continue
                    if n.split(".")[-1] == "cam_mount" and "head_frame" in gone:
                        continue
                    if overlap(t, s) > TOL:
                        blockers.append(n)
                out.append((f"{side + ' ' if side else ''}{label}", off, blockers))
    return out


def main():
    quick = "--quick" in sys.argv
    body, leg = B.Body(), LEG.Leg()
    bad = 0
    print("static fit, whole robot at the zero pose:")
    hits = collisions(body, leg, {})
    for h in hits:
        print("   ", h)
    bad += len(hits)
    print(f"  {len(hits)} overlapping pairs")
    moving = [n for n in assemble(body, leg, {}) if link_of(n, body, leg).split(".")[-1] in
              ("shoulder_link", "upper_arm", "forearm", "neck_link", "head_frame")]
    worst = {}
    ps = sweep_poses(quick)
    for i, q in enumerate(ps):
        for a, b, v in collisions(body, leg, q, names=moving, against=list(assemble(body, leg, q))):
            key = tuple(sorted((a, b)))
            if key not in worst or v > worst[key][0]:
                worst[key] = (v, q)
        print(f"\r  sweep pose {i + 1}/{len(ps)}  colliding pairs so far: {len(worst)}", end="", flush=True)
    print()
    for (a, b), (v, q) in worst.items():
        print(f"    {a} x {b}: {v} mm3 at {q}")
    bad += len(worst)
    blocked = 0
    for label, off, blockers in tool_access(body, leg):
        if blockers:
            blocked += 1
            print(f"  tool {label}: BLOCKED by {', '.join(blockers)}" + (f" (with {off} off)" if off else ""))
    print(f"  tool access: {'all screws reachable' if not blocked else str(blocked) + ' blocked'}")
    bad += blocked
    for jn, hit in service_paths(body, leg).items():
        print(f"  service {jn:15s} servo slides out along its shaft: {'OK' if hit is None else 'BLOCKED by ' + hit}")
        bad += hit is not None
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())


def service_paths(body, leg):
    """Each upper-body servo (with its horn) slides out along its shaft once its driven link (and everything after
    it) is off, in the service pose of that joint. {joint: None or what blocks it}."""
    out = {}
    for jn, j in body.joints.items():
        q = SERVICE_POSE.get(jn, {})
        comps = assemble(body, leg, q)
        side = "L." if jn in ("shoulder_pitch", "shoulder_roll", "elbow") else ""
        chain = ["shoulder_link", "upper_arm", "forearm"] if side else ["neck_link", "head_frame"]
        gone = set(chain[chain.index(j.driven):])
        mov = [comps[f"{side}servo_{jn}"], comps[f"{side}horn_{jn}"]]
        if side:
            T = chain_T(body, ARM_CHAIN, {k[2:]: v for k, v in q.items() if k.startswith("l_")})
            a = np.asarray(j.a, float)
            if j.holder != "shoulder_mount":
                tr = T[j.holder].wrapped.Transformation()
                R_ = np.array([[tr.Value(r, c) for c in (1, 2, 3)] for r in (1, 2, 3)])
                a = R_ @ a
        else:
            a = np.asarray(j.a, float)
        fixed = {n: s for n, s in comps.items() if not (link_of(n, body, leg).split(".")[-1] in gone and
                 (not side or n.startswith("L.")) or n.split(".")[-1] == "cam_mount" or n == "board_DISP")
                 and n not in (f"{side}servo_{jn}", f"{side}horn_{jn}")}
        out[jn] = None
        for d in np.arange(1.0, 40.0, 1.0):
            loc = cq.Location(cq.Vector(*(d * a)))
            hit = next((n for n, s in fixed.items() for m in mov if overlap(m.moved(loc), s) > TOL), None)
            if hit:
                out[jn] = f"{hit} after {d:.0f} mm"
                break
    return out
