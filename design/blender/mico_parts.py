"""Mico v2.0 shell, one source for the Blender model (mico_blender.py) and the mass report (mico_mass.py).

Every part is a superquadric: a rounded box whose corner roundness is set by two exponents
(e1 top-to-bottom, e2 around the sides; 1.0 = ellipsoid, 0.2 = almost a sharp box).
Positions are in millimetres, world frame, stand pose (x forward, y left, z up, sole on z = 0),
written as an offset from the link frame the part is bolted to. Link frames come from
sim/mini_humanoid/gen_robot.py, so joints, servos and the shell can never drift apart.

Mass model of a part (`mass` field):
    ("shell", wall_mm, material)   printed hollow part: outer solid minus a solid shrunk by the wall
    ("solid", material)            filled part (sole: infill density, see MATERIALS)
    ("mass", grams)                bought part, uniform box at its size (servos, battery, boards)
    None                           paint / decal / light, drawn but weightless
"""
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
SIM = REPO / "software" / "ai" / "sim" / "mini_humanoid"
sys.path.insert(0, str(SIM))
import gen_robot  # noqa: E402

# g/mm^3. Effective densities for printed parts include walls; infill only matters for "solid".
MATERIALS = {
    "PLA": 1.24e-3,
    "LW-PLA": 0.75e-3,       # foaming PLA printed at ~60% flow, same wall thickness
    "PETG": 1.27e-3,         # clear helmet, thermoformed from 0.8 mm sheet or printed in clear PETG
    "TPU-sole": 0.45e-3,     # sole printed in TPU 95A, 2 walls + 15% gyroid infill
}

WALL = 1.2       # torso, pelvis, head core
WALL_LIMB = 1.0  # arms, legs, joint covers
WALL_DOME = 0.8  # clear helmet


@dataclass
class Part:
    name: str
    link: str            # gen_robot body name ("base", "l_knee", ...)
    size: tuple          # full extents along its own x, y, z (mm)
    off: tuple           # centre offset from the link frame (mm)
    look: str            # material key used by Blender
    mass: tuple | None
    e: tuple = (0.5, 0.5)     # (e1, e2) roundness
    axis: str = "z"           # which world axis the part's own z runs along
    taper: float = 0.0        # +0.1: top 10% wider than the middle in x and y


# ---------------------------------------------------------------- link frames (stand pose)
def link_origins():
    out = {}

    def rec(b, p):
        w = tuple(a + c for a, c in zip(p, b.pos))
        out[b.name] = w
        for c in b.children:
            rec(c, w)
    rec(gen_robot.ROBOT, (0, 0, 0))
    return out


ORIGIN = link_origins()


def world(p: Part):
    return np.add(ORIGIN[p.link], p.off)


# ---------------------------------------------------------------- the shell (Design C, Balanced Cute)
HEAD = dict(size=(70, 80.5, 66), off=(0, 0, 23), e=(0.62, 0.62),   # 6% wider and deeper than v2.0, top at 303 mm
            taper=0.06)                                            # jaw 6% narrower: softer chin, neck clearance
VISOR = dict(size=(12, 62, 40), off=(29.8, 0, 21), e=(0.45, 0.5))  # navy face display window
EYE = dict(y=11.0, z=23.0, ry=5.2, rz=6.8)                         # default oval eye on the visor (mm, head frame)
MOUTH_Z = 10.5


def parts():
    P = []
    a = P.append
    sh = ("shell", WALL, "PLA")
    sl = ("shell", WALL_LIMB, "PLA")

    # torso, waist, pelvis ------------------------------------------------------------
    a(Part("torso", "base", (52, 72, 76), (0, 0, 74), "white", sh, (0.5, 0.5)))   # square, 3 mm clear of the arms
    a(Part("waist", "base", (40, 64, 9), (-1, 0, 33), "joint", sl, (0.35, 0.6)))           # abdomen transition
    a(Part("pelvis", "base", (46, 64, 34), (-2, 0, 14), "white", sh, (0.4, 0.5)))          # holds both hip-yaw servos
    a(Part("chest_panel", "base", (4, 34, 22), (25.0, 0, 74), "grey", None, (0.35, 0.35)))
    a(Part("chest_screen", "base", (2, 22, 9), (27.0, -2, 77), "visor", None, (0.3, 0.3)))
    a(Part("status_led", "base", (2, 4, 4), (27.0, 12, 77), "cyan", None, (1, 1)))
    a(Part("flag", "base", (1.6, 16, 11), (25.0, -20, 98), "red", None, (0.25, 0.25)))
    a(Part("back_panel", "base", (4, 40, 36), (-25.0, 0, 72), "grey", None, (0.35, 0.35)))  # electronics access
    a(Part("back_switch", "base", (2, 10, 5), (-27.2, 4, 80), "joint", None, (0.4, 0.4)))
    a(Part("collar", "base", (34, 42, 7), (0, 0, 107), "orange", sl, (0.6, 1.0)))
    a(Part("neck_housing", "neck_yaw", (22, 28, 15), (0, 0, 9), "joint", sl, (0.55, 0.7)))  # head-pitch servo cover

    # head ----------------------------------------------------------------------------
    a(Part("head", "head_pitch", HEAD["size"], HEAD["off"], "white", sh, HEAD["e"], taper=HEAD["taper"]))
    a(Part("visor", "head_pitch", VISOR["size"], VISOR["off"], "visor", sl, VISOR["e"]))
    a(Part("camera_window", "head_pitch", (4, 22, 7), (31.6, 0, 44), "window", None, (0.4, 0.4)))
    for s in (1, -1):
        a(Part(f"ear_{s}", "head_pitch", (18, 7, 18), (0, 41.5 * s, 22), "orange", sl, (0.3, 1.0), "y"))
        a(Part(f"ear_grille_{s}", "head_pitch", (12, 1.2, 12), (0, 45.2 * s, 22), "joint", None, (0.1, 1.0), "y"))

    # arms ----------------------------------------------------------------------------
    for side, s in (("l", 1), ("r", -1)):
        a(Part(f"{side}_shoulder", f"{side}_shoulder_pitch", (34, 34, 34), (0, 8 * s, -2), "white", sl, (0.35, 1.0),
               "y"))                                                       # actuator housing on the pitch axis
        a(Part(f"{side}_shoulder_cap", f"{side}_shoulder_pitch", (24, 1.6, 24), (0, 25.6 * s, -2), "orange", None,
               (0.2, 1.0), "y"))
        a(Part(f"{side}_upper_arm", f"{side}_shoulder_roll", (20, 20, 32), (0, 0, -28), "white", sl, (0.5, 0.9)))
        a(Part(f"{side}_elbow", f"{side}_elbow", (24, 22, 24), (0, 0, 0), "joint", sl, (0.3, 1.0), "y"))
        a(Part(f"{side}_forearm", f"{side}_elbow", (19, 19, 30), (0, 0, -25), "white", sl, (0.5, 0.9), taper=0.08))
        a(Part(f"{side}_wrist", f"{side}_elbow", (16, 16, 6), (0, 0, -40), "joint", None, (0.4, 1.0)))
        a(Part(f"{side}_hand", f"{side}_elbow", (16, 12, 18), (0, 0, -47), "grey", sl, (0.5, 0.7)))
        a(Part(f"{side}_thumb", f"{side}_elbow", (6, 6, 10), (7, -2 * s, -44), "grey", sl, (0.6, 1.0)))

    # legs ----------------------------------------------------------------------------
    for side, s in (("l", 1), ("r", -1)):
        # hip and ankle covers sit on the leg axis, as wide as the thigh: the leg reads as one straight column
        a(Part(f"{side}_hip", f"{side}_hip_roll", (28, 26, 30), (0, 0, -4), "joint", sl, (0.55, 0.7)))
        a(Part(f"{side}_thigh", f"{side}_hip_pitch", (28, 26, 40), (0, 0, -24), "white", sl, (0.5, 0.85)))
        a(Part(f"{side}_knee", f"{side}_knee", (26, 22, 26), (0, 0, 0), "joint", sl, (0.3, 1.0), "y"))
        a(Part(f"{side}_knee_pad", f"{side}_knee", (4, 12, 10), (12.6, 0, 0), "orange", None, (0.4, 0.5)))
        a(Part(f"{side}_shin", f"{side}_knee", (26, 24, 34), (0, 0, -25), "white", sl, (0.5, 0.85)))
        a(Part(f"{side}_ankle", f"{side}_ankle_pitch", (28, 26, 18), (0, 0, 4), "joint", sl, (0.55, 0.8)))
        a(Part(f"{side}_shoe", f"{side}_ankle_roll", (64, 33, 16), (0, 0, -13), "white", sl, (0.5, 0.6)))
        # boot collar: carries the shoe up into the ankle cuff, so foot and leg read as one piece
        a(Part(f"{side}_boot", f"{side}_ankle_roll", (28, 26, 15), (0, 0, -6), "white", sl, (0.55, 0.8)))
        a(Part(f"{side}_heel_tab", f"{side}_ankle_roll", (5, 16, 7), (-32.3, 0, -13), "orange", None,
               (0.4, 0.5)))
        a(Part(f"{side}_sole", f"{side}_ankle_roll", (76, 38, 8), (0, 0, -21), "navy",
               ("solid", "TPU-sole"), (0.3, 0.55)))
    return P


# ---------------------------------------------------------------- bought parts and allowances
# Everything gen_robot models that is not shell (servos, battery, boards, wiring) is reused as is.
SHELL_COLOURS = {"shell", "frame", "lens"}


def components():
    """(name, link, size mm, world centre mm, grams, kind)"""
    out = []
    for b in gen_robot.walk(gen_robot.ROBOT):
        for g in b.geoms:
            if g.color in SHELL_COLOURS or g.mass <= 0:
                continue
            kind = "servo" if g.name.endswith("_servo") else g.color
            pos = np.add(ORIGIN[b.name], g.pos)
            if g.name == "camera":                     # camera moved up into the forehead pod (camera_link)
                pos = np.add(ORIGIN[b.name], (24, 0, 42))
                g = gen_robot.Geom("camera", (10, 25, 24), (0, 0, 0), g.mass, g.color)
            out.append((g.name, b.name, g.size, pos, g.mass, kind))

    def add(name, link, size, off, grams, kind="extra"):
        out.append((name, link, size, np.add(ORIGIN[link], off), grams, kind))

    # Mico v2.0 additions that are not in gen_robot yet
    add("face_oled", "head_pitch", (8, 60, 30), (17, 0, 22), 12.0)       # 2.42" OLED module + driver
    add("tof_vl53l1x", "head_pitch", (5, 13, 18), (28, 8, 45), 1.5)
    for side, s in (("l", 1), ("r", -1)):
        add(f"{side}_mic", "head_pitch", (8, 3, 8), (0, 40 * s, 22), 1.0)
        add(f"{side}_fsr_pair", f"{side}_ankle_roll", (60, 20, 1), (10, 6 * s, -24), 2.0)
    # printed servo brackets, horns and screws: 3 g per leg servo, 1.5 g per arm / neck servo,
    # placed on the link that carries the servo horn side
    for b in gen_robot.walk(gen_robot.ROBOT):
        if b.joint is None:
            continue
        heavy = any(k in b.name for k in ("hip", "knee", "ankle"))
        add(f"{b.name}_bracket", b.name, (24, 24, 24), (0, 0, 0), 3.0 if heavy else 1.5)
    return out


# ---------------------------------------------------------------- geometry
def _sgnpow(v, p):
    return np.sign(v) * np.abs(v) ** p


def superquadric(size, e=(0.5, 0.5), n_lat=48, n_lon=96, taper=0.0):
    """Closed triangle mesh (V in mm, centred at 0, outward normals). Poles are single vertices."""
    ax, ay, az = (s / 2 for s in size)
    e1, e2 = e
    eta = np.linspace(-np.pi / 2, np.pi / 2, n_lat + 1)[1:-1]
    om = np.linspace(-np.pi, np.pi, n_lon, endpoint=False)
    E, W = np.meshgrid(eta, om, indexing="ij")
    ce, se = _sgnpow(np.cos(E), e1), _sgnpow(np.sin(E), e1)
    x = ax * ce * _sgnpow(np.cos(W), e2)
    y = ay * ce * _sgnpow(np.sin(W), e2)
    z = az * se
    k = 1 + taper * z / az
    V = np.column_stack([(x * k).ravel(), (y * k).ravel(), z.ravel()])
    V = np.vstack([V, [0, 0, -az], [0, 0, az]])
    bot, top = len(V) - 2, len(V) - 1
    nr = n_lat - 1
    F = []
    idx = lambda i, j: i * n_lon + (j % n_lon)   # noqa: E731
    for i in range(nr - 1):
        for j in range(n_lon):
            a, b, c, d = idx(i, j), idx(i, j + 1), idx(i + 1, j + 1), idx(i + 1, j)
            F += [(a, b, c), (a, c, d)]
    for j in range(n_lon):
        F.append((bot, idx(0, j + 1), idx(0, j)))
        F.append((top, idx(nr - 1, j), idx(nr - 1, j + 1)))
    return V, np.array(F)


AXIS_ROT = {
    "z": np.eye(3),
    "x": np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]),    # own z -> world x
    "y": np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]),    # own z -> world y
}


def own_size(p: Part):
    """The part's size is given in world axes; convert to its own axes before rotating."""
    sx, sy, sz = p.size
    return {"z": (sx, sy, sz), "x": (sz, sy, sx), "y": (sx, sz, sy)}[p.axis]


def part_mesh(p: Part, shrink=0.0, **kw):
    size = tuple(max(s - 2 * shrink, 0.1) for s in own_size(p))
    V, F = superquadric(size, p.e, taper=p.taper, **kw)
    V = V @ AXIS_ROT[p.axis].T + world(p)
    return V, F


# ---------------------------------------------------------------- decals that are not superquadrics
def _star(r_out, r_in, depth):
    """Five-point star facing +x, drawn in the y-z plane, closed triangle mesh (mm)."""
    pts = []
    for i in range(10):
        r = r_out if i % 2 == 0 else r_in
        a = np.pi / 2 + i * np.pi / 5
        pts.append((-r * np.cos(a), r * np.sin(a)))
    V = [(depth / 2, y, z) for y, z in pts] + [(-depth / 2, y, z) for y, z in pts]
    V += [(depth / 2, 0, 0), (-depth / 2, 0, 0)]
    cf, cb = 20, 21
    F = []
    for i in range(10):
        j = (i + 1) % 10
        F += [(cf, j, i), (cb, 10 + i, 10 + j), (i, 10 + j, 10 + i), (i, j, 10 + j)]
    return np.array(V, float), np.array(F)


def _tube(points, radius, seg=12):
    """Swept circle along a polyline (mm), capped, triangles."""
    P = [np.asarray(p, float) for p in points]
    V, F = [], []
    for i, p in enumerate(P):
        t = P[min(i + 1, len(P) - 1)] - P[max(i - 1, 0)]
        t /= np.linalg.norm(t)
        n = np.cross(t, [1, 0, 0]) if abs(t[0]) < 0.9 else np.cross(t, [0, 0, 1])
        n /= np.linalg.norm(n)
        b = np.cross(t, n)
        for k in range(seg):
            a = 2 * np.pi * k / seg
            V.append(p + radius * (np.cos(a) * n + np.sin(a) * b))
    for i in range(len(P) - 1):
        for k in range(seg):
            a, b = i * seg + k, i * seg + (k + 1) % seg
            F += [(a, b, b + seg), (a, b + seg, a + seg)]
    n0, n1 = len(V), len(V) + 1
    V += [P[0], P[-1]]
    last = (len(P) - 1) * seg
    for k in range(seg):
        F += [(n0, (k + 1) % seg, k), (n1, last + k, last + (k + 1) % seg)]
    return np.array(V), np.array(F)


def visor_front(y, z):
    """x of the visor's front surface at (y, z), head frame (mm)."""
    a, b, c = (v / 2 for v in VISOR["size"])
    e1, e2 = VISOR["e"]
    cx, _, cz = VISOR["off"]
    t = max(1 - abs((z - cz) / c) ** (2 / e1), 0) ** (e1 / e2) - abs(y / b) ** (2 / e2)
    return cx + a * max(t, 0) ** (e2 / 2)


def _ellipse(cy, cz, ry, rz, n=56):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.column_stack([cy + ry * np.cos(t), cz + rz * np.sin(t)])


def _clip(poly, a, b, c):
    """Keep the part of a convex polygon where a*y + b*z <= c (Sutherland-Hodgman, one edge)."""
    out = []
    for i in range(len(poly)):
        p, q = poly[i], poly[(i + 1) % len(poly)]
        fp, fq = a * p[0] + b * p[1] - c, a * q[0] + b * q[1] - c
        if fp <= 0:
            out.append(p)
        if fp * fq < 0:
            out.append(p + (q - p) * fp / (fp - fq))
    return np.array(out)


def _plate(poly, x, depth=0.5):
    """Convex polygon in the y-z plane extruded along x, centred on x, closed and outward facing."""
    n = len(poly)
    c = poly.mean(axis=0)
    V = [(x + depth / 2, y, z) for y, z in poly] + [(x - depth / 2, y, z) for y, z in poly]
    V += [(x + depth / 2, *c), (x - depth / 2, *c)]
    cf, cb = 2 * n, 2 * n + 1
    F = []
    for i in range(n):
        j = (i + 1) % n
        F += [(cf, i, j), (cb, n + j, n + i), (i, n + i, n + j), (i, n + j, j)]
    V, F = np.array(V, float), np.array(F)
    if np.cross(V[F[0][1]] - V[F[0][0]], V[F[0][2]] - V[F[0][0]])[0] < 0:     # front fan must face +x
        F = F[:, ::-1]
    return V, F


EXPRESSIONS = ("neutral", "happy", "curious", "sleepy", "surprised", "sad")


def face(expr="neutral"):
    """Display layers on the visor for one expression: [(name, look, V head-frame mm, F)].
    Four layers per eye: glow, cyan oval, navy pupil, white highlight. Only `neutral` is physical shape
    reference; the others are frames for the OLED animation."""
    out = []
    zc = EYE["z"]

    def layer(name, look, poly, k, yc):
        x = visor_front(yc, zc) + 0.15 + 0.3 * k
        out.append((name, look, *_plate(poly, x)))

    def tube(name, pts, r):
        pts = [(visor_front(y, z) + 0.9, y, z) for y, z in pts]
        out.append((name, "cyan", *_tube(pts, r, seg=10)))

    for s in (1, -1):
        yc = EYE["y"] * s
        ry, rz, py, pz = EYE["ry"], EYE["rz"], 0.0, -0.6
        pry, prz, glint = 2.7, 3.6, True
        if expr == "surprised":
            ry, rz, pry, prz = ry * 1.15, rz * 1.15, 2.0, 2.6
        if expr == "curious":
            k = 1.12 if s > 0 else 0.92
            ry, rz, pry, prz = ry * k, rz * k, pry * k, prz * k
            py, pz = 1.4, 0.8
        if expr == "sad":
            pz = -1.8
        if expr == "happy":                                      # closed, smiling arcs
            arc = [(yc + 5.0 * np.cos(t), zc - 1.5 + 4.6 * np.sin(t)) for t in np.linspace(0.15, np.pi - 0.15, 15)]
            tube(f"eye_arc_{s}", arc, 1.45)
            continue
        cut = None
        if expr == "sleepy":
            cut = (0.0, 1.0, zc + 0.6)                           # lid straight across, just above the centre
        if expr == "sad":
            cut = (0.42 * s, 1.0, zc + 3.2 + 0.42 * s * yc)      # lid slopes down towards the outside
        shapes = [("glow", "eye_glow", _ellipse(yc, zc, ry + 1.3, rz + 1.3)),
                  ("oval", "cyan", _ellipse(yc, zc, ry, rz)),
                  ("pupil", "pupil", _ellipse(yc + py * s if expr != "curious" else yc + py, zc + pz, pry, prz))]
        if glint and expr != "sleepy":
            shapes.append(("glint", "glint", _ellipse(yc - 1.7, zc + 2.4, 1.25, 1.25, 24)))
            shapes.append(("glint2", "glint", _ellipse(yc + 1.4, zc - 2.6, 0.55, 0.55, 16)))
        for k, (n, look, poly) in enumerate(shapes):
            if cut is not None:
                poly = _clip(poly, *cut)
                if len(poly) < 3:
                    continue
            layer(f"eye_{n}_{s}", look, poly, k, yc)

    mz = MOUTH_Z
    if expr in ("neutral", "curious"):
        w, d, dy = 3.2, 1.3, (0.8 if expr == "curious" else 0.0)
        tube("mouth", [(dy + w * np.sin(t), mz - d * np.cos(t) + d) for t in np.linspace(-1.2, 1.2, 11)], 0.75)
    elif expr == "happy":
        tube("mouth", [(4.2 * np.sin(t), mz - 2.4 * np.cos(t) + 1.6) for t in np.linspace(-1.25, 1.25, 13)], 0.8)
    elif expr == "sleepy":
        tube("mouth", [(-2.0, mz + 0.6), (2.0, mz + 0.6)], 0.7)
    elif expr == "surprised":
        tube("mouth", [(1.9 * np.cos(t), mz + 0.4 + 2.2 * np.sin(t)) for t in np.linspace(0, 2 * np.pi, 21)], 0.7)
    elif expr == "sad":
        tube("mouth", [(3.0 * np.sin(t), mz + 1.6 * np.cos(t) - 0.4) for t in np.linspace(-1.2, 1.2, 11)], 0.75)
    return out


def decals(expr="neutral"):
    """(name, link, look, V world mm, F): flag star, chest ECG trace and the face on the visor."""
    b, h = np.array(ORIGIN["base"]), np.array(ORIGIN["head_pitch"])
    V, F = _star(4.0, 1.6, 0.8)
    out = [("flag_star", "base", "yellow", V + b + (26.1, -20, 98), F)]
    ecg = [(-10, 0), (-5, 0), (-3, 2.5), (-1, -3), (1, 3.2), (3, 0), (10, 0)]
    out.append(("ecg_trace", "base", "cyan", *_tube([b + (28.2, -2 + y, 77 + z) for y, z in ecg], 0.5)))
    for name, look, V, F in face(expr):
        out.append((name, "head_pitch", look, V + h, F))
    return out
