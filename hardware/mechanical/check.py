"""Interference check for the leg: no two parts of different links may overlap anywhere in the joint ranges, and
every servo must sit in its pocket without touching the walls.

    python check.py            # prints a report, exits 1 on a collision
    python check.py --quick    # fewer poses (CI)

Poses: each joint swept alone over its full range, plus samples of the demo gait (src/humanoid_control/scripts/
mico_gait.py) and a few combined extremes. Parts that touch by design (servo ears on their bosses, horn in its
recess, the idler boss on the bearing race) are compared with a 0.05 mm³ tolerance.
"""
import itertools
import sys
from pathlib import Path

import numpy as np

import leg as L
import pose as P

REPO = Path(__file__).resolve().parent
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
sys.path.insert(0, str(REPO / "software" / "ros2" / "src" / "humanoid_control" / "scripts"))

RANGES = {"hip_yaw": (-45, 45), "hip_roll": (-20, 20), "hip_pitch": (-80, 40), "knee": (0, 120),
          "ankle_pitch": (-45, 45),
          "ankle_roll": (-20, 20)}
TOL = 0.05   # mm³


def gait_poses(n):
    import mico_gait as G
    out = []
    for t in np.linspace(0, G.T_END, n, endpoint=False):
        q = G.targets(t)
        out.append({j: float(np.degrees(q.get(f"l_{j}", 0.0))) for j in RANGES})
    return out


def poses(quick):
    k = 4 if quick else 7
    out = [{}]
    for j, (lo, hi) in RANGES.items():
        out += [{j: float(v)} for v in np.linspace(lo, hi, k)]
    out += gait_poses(12 if quick else 40)
    out += [{"hip_pitch": -60, "knee": 110, "ankle_pitch": -45, "hip_roll": 20, "ankle_roll": -20},
            {"hip_pitch": 30, "knee": 0, "ankle_pitch": 45, "hip_roll": -20, "ankle_roll": 20},
            {"hip_pitch": -80, "knee": 120, "ankle_pitch": -40}]
    return out


def overlap(a, b):
    ba, bb = a.BoundingBox(), b.BoundingBox()
    if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin
            or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
        return 0.0
    return a.intersect(b).Volume()


def service_paths(leg):
    """Each servo (with its horn) must slide out along its shaft once its driven link is off, without touching
    anything that stays on the robot. Returns {joint: first blocking part or None}."""
    import cadquery as cq
    comps = P.posed(leg, {}, include=("parts", "servos", "horns"))
    out = {}
    for jn, j in leg.joints.items():
        if j.holder == "hip_mount":
            continue                                   # the yaw servo drops out downward: covered by tool_access
        moving = [comps[f"servo_{jn}"], comps[f"horn_{jn}"]]
        fixed = {n: s for n, s in comps.items() if n not in (f"servo_{jn}", f"horn_{jn}", j.driven)
                 and P.owner(leg, n) != j.driven or n.startswith("servo_") and n != f"servo_{jn}"}
        fixed.pop(j.driven, None)
        out[jn] = None
        for d in np.arange(1.0, 40.0, 1.0):
            loc = cq.Location(cq.Vector(*(d * np.asarray(j.a, float))))
            hit = next((n for n, s in fixed.items() for m in moving if overlap(m.moved(loc), s) > TOL), None)
            if hit:
                out[jn] = f"{hit} after {d:.0f} mm"
                break
    return out


def tool_access(leg):
    """Every screw can be reached with a straight tool once the parts listed for it are off."""
    comps = P.posed(leg, {}, include=("parts", "servos", "horns"))
    out = []
    for label, tool, off in leg.tool_paths():
        gone = removed(off) if off else set()
        blockers = [n for n, s in comps.items() if not (n in gone or P.owner(leg, n) in gone)
                    and overlap(tool, s) > TOL]
        out.append((label, off, blockers))
    return out


DOWNSTREAM = ["yaw_bracket", "hip_block", "thigh", "shin", "ankle_block", "foot"]


def removed(parts):
    """Taking a link off a joint takes everything below it with it. `parts` may join several with '+'."""
    out = set()
    for part in parts.split("+"):
        out |= set(DOWNSTREAM[DOWNSTREAM.index(part):]) if part in DOWNSTREAM else {part}
    return out


def two_legs(leg):
    """Both legs on the pelvis at y = ±HIP_Y: one yaw from 15° toe-in to 30° toe-out with the other leg straight (the
    gait turns one leg up to 15° toe-out), both yaws together up to ±12°, roll as in a weight shift (both legs tilt the
    same way, ±17°). Beyond that (both feet yawed the same way more than 12°) the centred feet meet: the controller
    must keep the legs apart."""
    import cadquery as cq
    hits = []
    names = [n for n in list(leg.parts) + list(leg.servos()) if P.owner(leg, n) != "pelvis"]
    yaws = [(y, 0) for y in (-15, 0, 30)] + [(0, y) for y in (-15, 30)] + [(12, 12), (-12, -12)]
    for yl, yr in yaws:
        for rl, rr in ((0, 0), (17, -17), (-17, 17)):
            left = P.posed(leg, {"hip_yaw": yl, "hip_roll": rl})
            right = P.posed(leg, {"hip_yaw": yr, "hip_roll": rr})
            L_ = {n: left[n].moved(cq.Location(cq.Vector(0, L.HIP_Y, 0))) for n in names}
            R_ = {n: right[n].mirror("XZ").moved(cq.Location(cq.Vector(0, -L.HIP_Y, 0))) for n in names}
            for a in names:
                for b in names:
                    v = overlap(L_[a], R_[b])
                    if v > TOL:
                        hits.append((a, b, v, (yl, yr, rl, rr)))
    return hits


def main():
    quick = "--quick" in sys.argv
    leg = L.Leg()
    names = list(leg.parts) + list(leg.servos()) + list(leg.horns())
    owner = {n: P.owner(leg, n) for n in names}
    hits = {}
    ps = poses(quick)
    for i, q in enumerate(ps):
        comps = P.posed(leg, q, include=("parts", "servos", "horns"))
        for a, b in itertools.combinations(names, 2):
            if a.startswith(("servo_", "horn_")) and b.startswith(("servo_", "horn_")) and \
                    a.split("_", 1)[1] == b.split("_", 1)[1]:
                continue                                  # a horn sits on its own servo's spline
            same = owner[a] == owner[b]
            if same and not (a.startswith("servo_") or b.startswith("servo_")):
                continue
            v = overlap(comps[a], comps[b])
            if v > TOL:
                key = (a, b)
                if key not in hits or v > hits[key][0]:
                    hits[key] = (v, q)
        print(f"\rpose {i + 1}/{len(ps)}  collisions so far: {len(hits)}", end="", flush=True)
    print()
    for (a, b), (v, q) in sorted(hits.items(), key=lambda kv: -kv[1][0]):
        print(f"  {a:24s} x {b:24s} {v:9.2f} mm3  at {q}")
    print(f"{len(ps)} poses, {len(hits)} colliding pairs")
    paths = service_paths(leg)
    for jn, hit in paths.items():
        print(f"  service {jn:12s} servo slides out along its shaft: {'OK' if hit is None else 'BLOCKED by ' + hit}")
    bad = 0
    for label, off, blockers in tool_access(leg):
        if blockers:
            bad += 1
            print(f"  tool  {label}: BLOCKED by {', '.join(blockers)}" + (f" (with {off} off)" if off else ""))
    print(f"  tool access: {len(leg.tool_paths()) - bad} of {len(leg.tool_paths())} screws reachable")
    legs = two_legs(leg)
    worst = {}
    for a, b, v, q in legs:
        if (a, b) not in worst or v > worst[(a, b)][0]:
            worst[(a, b)] = (v, q)
    for (a, b), (v, q) in worst.items():
        print(f"  legs  L.{a} x R.{b}: {v:.1f} mm3 at yaw/roll (L, R, L, R) {q}")
    print(f"  two legs at y = ±{L.HIP_Y}: {'clear' if not legs else str(len(worst)) + ' colliding pairs'}")
    return 1 if hits or any(paths.values()) or bad or legs else 0


if __name__ == "__main__":
    sys.exit(main())
