"""Pose the leg: every link and bought part placed for a set of joint angles (degrees)."""
import cadquery as cq
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf

CHAIN = [  # (joint, parent link, child link)
    ("hip_yaw", "pelvis", "yaw_bracket"),
    ("hip_roll", "yaw_bracket", "hip_block"),
    ("hip_pitch", "hip_block", "thigh"),
    ("knee", "thigh", "shin"),
    ("ankle_pitch", "shin", "ankle_block"),
    ("ankle_roll", "ankle_block", "foot"),
]
# kinematic joint axis (sign as in the URDF): hip yaw +z, roll +x, pitch +y
AXIS = {"hip_yaw": (0, 0, 1), "hip_roll": (1, 0, 0), "hip_pitch": (0, 1, 0), "knee": (0, 1, 0),
        "ankle_pitch": (0, 1, 0), "ankle_roll": (1, 0, 0)}
LINK_OF = {"sole": "foot", "hip_mount": "pelvis", "thrust_ring": "pelvis"}


def link_locations(leg, q):
    """World location of each link frame for joint angles q (deg)."""
    import math
    T = {"pelvis": gp_Trsf()}
    for jn, parent, child in CHAIN:
        j = leg.joints[jn]
        o = j.origin
        rot = gp_Trsf()
        rot.SetRotation(gp_Ax1(gp_Pnt(*map(float, o)), gp_Dir(*map(float, AXIS[jn]))), math.radians(q.get(jn, 0.0)))
        t = T[parent].Multiplied(rot)
        T[child] = t
    return {k: cq.Location(v) for k, v in T.items()}


def owner(leg, name):
    """Which link a component moves with."""
    if name in leg.parts:
        return LINK_OF.get(name, name)
    kind, jn = name.split("_", 1)[0], name.split("_", 1)[1]
    jn = jn.rsplit("_horn", 1)[0].rsplit("_idler", 1)[0] if kind == "cap" else jn
    j = leg.joints[jn]
    link = j.holder if kind == "servo" else j.driven
    return LINK_OF.get(link, link)


def posed(leg, q, include=("parts", "servos", "horns", "caps")):
    locs = link_locations(leg, q)
    comps = {}
    if "parts" in include:
        comps.update(leg.parts)
    if "servos" in include:
        comps.update(leg.servos())
    if "horns" in include:
        comps.update(leg.horns())
    if "caps" in include:
        comps.update(leg.caps())
    return {n: s.moved(locs[owner(leg, n)]) for n, s in comps.items()}
