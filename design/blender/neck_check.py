"""Clearance between the head shell and the collar / torso over the neck range (Design C).

    python neck_check.py

The head is swept through neck yaw x head pitch at the gen_robot joint limits. For each pose the
smallest gap is found by growing the head outward until it first touches the torso or the collar
(analytic superquadric inside-test, torso taper included).
"""
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mico_parts as mp  # noqa: E402

J = {b.joint.name: b.joint.range_deg for b in mp.gen_robot.walk(mp.gen_robot.ROBOT) if b.joint}
P = {p.name: p for p in mp.parts()}


def inside(p, X):
    """Superquadric inside test (value < 1 inside) for world points X (mm); axis-z parts only."""
    c = mp.world(p)
    ax, ay, az = (s / 2 for s in p.size)
    e1, e2 = p.e
    x, y, z = (X - c).T
    k = 1 + p.taper * np.clip(z / az, -1, 1)
    x, y = x / k, y / k
    return (np.abs(x / ax) ** (2 / e2) + np.abs(y / ay) ** (2 / e2)) ** (e2 / e1) + np.abs(z / az) ** (2 / e1)


def rot(axis, deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return {"y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]), "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def head_points(grow, pitch, yaw):
    head = P["head"]
    V, _ = mp.part_mesh(replace(head, size=tuple(s + 2 * grow for s in head.size)), n_lat=40, n_lon=80)
    hp, ny = np.array(mp.ORIGIN["head_pitch"]), np.array(mp.ORIGIN["neck_yaw"])
    V = (V - hp) @ rot("y", pitch).T + hp
    return (V - ny) @ rot("z", yaw).T + ny


def touching(grow, pitch, yaw):
    X = head_points(grow, pitch, yaw)
    return any((inside(P[n], X) < 1).any() for n in ("torso", "collar"))


def clearance(pitch, yaw):
    if touching(0, pitch, yaw):
        lo, hi = -20.0, 0.0              # already intersecting: how deep
        while hi - lo > 0.1:
            mid = (lo + hi) / 2
            lo, hi = (lo, mid) if touching(mid, pitch, yaw) else (mid, hi)
        return lo
    lo, hi = 0.0, 30.0
    while hi - lo > 0.1:
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if not touching(mid, pitch, yaw) else (lo, mid)
    return lo


def main():
    pitches = np.linspace(*J["head_pitch"], 16)
    yaws = np.linspace(*J["neck_yaw"], 13)
    worst = min((clearance(p, y), p, y) for p in pitches for y in yaws)
    print(f"head pitch {J['head_pitch']} deg, neck yaw {J['neck_yaw']} deg")
    print(f"smallest head-to-body gap {worst[0]:.1f} mm at pitch {worst[1]:.0f} deg, yaw {worst[2]:.0f} deg")
    for p in (-30, 0, 20, 30, 45):
        print(f"  pitch {p:4d} deg, yaw 0: gap {clearance(p, 0):5.1f} mm")
    lim = max(p for p in np.arange(0, 46, 1) if all(clearance(p, y) >= 2.0 for y in yaws))
    print(f"head-down limit for a 2 mm gap at any yaw: {lim:.0f} deg")
    return worst, lim


if __name__ == "__main__":
    main()
