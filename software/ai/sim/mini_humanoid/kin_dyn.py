"""Kinematics + dynamics tables for Mico, the 20-DoF mini humanoid. Needs only numpy.

    python kin_dyn.py          # verify DH vs the URDF, write tables/*.csv, print a summary

1. DH tables (standard / distal convention):  T_i = Rz(theta_i) . Tz(d_i) . Tx(a_i) . Rx(alpha_i)
   theta_i = q_i + offset_i.  Lengths in mm, angles in degrees.
   Each chain starts from a fixed base transform measured from base_link, which sits
   at the hip point H (between both hips, 115 mm above the floor). Frame: x fwd, y left, z up.
2. Forward kinematics from the DH tables is checked against the URDF at random joint angles.
3. Link dynamics (mass, CoM, inertia) are read from humanoid.urdf.
4. Static joint loads are computed from gravity on each subtree: arms / head with the body upright, legs
   standing on one foot (the gait's single-support pose) and in a squat (two feet, half the weight each).
   Planned joint speed / acceleration come from the gait trajectory (mico_gait.py). Torques during the gait
   come from MuJoCo: run sim/mujoco/check_mujoco.py first and its tables/mj_servo_loads.csv is merged in.
"""
import csv
import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from gen_robot import SERVOS

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
GAIT_PATH = REPO / "software" / "ros2" / "src" / "humanoid_control" / "scripts" / "mico_gait.py"
_spec = importlib.util.spec_from_file_location("mico_gait", GAIT_PATH)
gait = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gait)

G = 9.81
OUT = HERE / "tables"

# ---------------------------------------------------------------- DH tables
# (joint, theta offset deg, d mm, a mm, alpha deg)


def leg_dh(s):
    return [
        (f"{s}_hip_yaw",     90, 0,  0,  90),
        (f"{s}_hip_roll",   -90, 0,  0, -90),
        (f"{s}_hip_pitch",    0, 0, 45,   0),
        (f"{s}_knee",         0, 0, 45,   0),
        (f"{s}_ankle_pitch",  0, 0,  0,  90),
        (f"{s}_ankle_roll",   0, 0, 25,   0),
    ]


def arm_dh(s):
    k = 1 if s == "l" else -1
    return [
        (f"{s}_shoulder_pitch", 0, 5 * k,  0,  90),
        (f"{s}_shoulder_roll",  0, 0,     50, -90),
        (f"{s}_elbow",          0, 0,     44,   0),
    ]


HEAD_DH = [
    ("neck_yaw",   0, 22, 0, -90),
    ("head_pitch", 0,  0, 0,   0),
]

# fixed base -> frame 0 of each chain (rotation columns = x0, y0, z0 in the base frame)
R_ARM0 = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]], float)   # x0 = -z, y0 = -x, z0 = +y
CHAINS = {
    "l_leg": dict(dh=leg_dh("l"), p0=(0, 22, 0), R0=np.eye(3), tool=None, tip="sole (below ankle)"),
    "r_leg": dict(dh=leg_dh("r"), p0=(0, -22, 0), R0=np.eye(3), tool=None, tip="sole (below ankle)"),
    "l_arm": dict(dh=arm_dh("l"), p0=(0, 40, 97), R0=R_ARM0, tool=None, tip="palm center"),
    "r_arm": dict(dh=arm_dh("r"), p0=(0, -40, 97), R0=R_ARM0, tool=None, tip="palm center"),
    "head":  dict(dh=HEAD_DH, p0=(0, 0, 110), R0=np.eye(3), tool=(28, -24, 0), tip="camera"),
}


def dh_T(theta, d, a, alpha):
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0, sa, ca, d],
                     [0, 0, 0, 1]])


def fk(chain, q):
    """Return list of frame origins (mm, base frame) for frames 0..n (+ tool)."""
    c = CHAINS[chain]
    T = np.eye(4)
    T[:3, :3] = c["R0"]
    T[:3, 3] = c["p0"]
    pts = [T[:3, 3].copy()]
    for (name, off, d, a, al) in c["dh"]:
        T = T @ dh_T(q[name] + math.radians(off), d, a, math.radians(al))
        pts.append(T[:3, 3].copy())
    if c["tool"] is not None:
        pts.append((T @ np.array([*c["tool"], 1.0]))[:3])
    return pts, T


# ---------------------------------------------------------------- URDF model
def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, a):
    x, y, z = axis
    c, s, t = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


def _vec(e, attr, default="0 0 0"):
    return np.array([float(v) for v in (e.get(attr, default) if e is not None else default).split()])


class Urdf:
    """Just enough URDF for FK, CoM and static loads (SI units)."""

    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.links = {}
        for ln in root.findall("link"):
            ine = ln.find("inertial")
            if ine is None:
                self.links[ln.get("name")] = (0.0, np.zeros(3), np.zeros(3))
                continue
            i = ine.find("inertia")
            self.links[ln.get("name")] = (float(ine.find("mass").get("value")), _vec(ine.find("origin"), "xyz"),
                                          np.array([float(i.get(k)) for k in ("ixx", "iyy", "izz")]))
        self.joints = {}
        self.children = {}
        for j in root.findall("joint"):
            o, lim = j.find("origin"), j.find("limit")
            T = np.eye(4)
            T[:3, :3] = _rpy(*_vec(o, "rpy"))
            T[:3, 3] = _vec(o, "xyz")
            jd = dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                      child=j.find("child").get("link"), T=T, axis=_vec(j.find("axis"), "xyz", "1 0 0"),
                      lower=float(lim.get("lower")) if lim is not None else 0.0,
                      upper=float(lim.get("upper")) if lim is not None else 0.0,
                      effort=float(lim.get("effort")) if lim is not None else 0.0)
            self.joints[jd["name"]] = jd
            self.children.setdefault(jd["parent"], []).append(jd)
        self.movable = [j for j in self.joints.values() if j["type"] == "revolute"]

    def fk(self, q, root=np.eye(4)):
        """World transform of every link, base_link at `root`."""
        X = {"base_link": root}
        stack = ["base_link"]
        while stack:
            p = stack.pop()
            for j in self.children.get(p, []):
                T = X[p] @ j["T"]
                if j["type"] == "revolute":
                    R = np.eye(4)
                    R[:3, :3] = _axis_angle(j["axis"], q.get(j["name"], 0.0))
                    T = T @ R
                X[j["child"]] = T
                stack.append(j["child"])
        return X

    def subtree(self, link):
        out, stack = [], [link]
        while stack:
            k = stack.pop()
            out.append(k)
            stack += [j["child"] for j in self.children.get(k, [])]
        return out

    def mass_com(self, X, links):
        M = sum(self.links[k][0] for k in links)
        c = sum(self.links[k][0] * (X[k] @ np.append(self.links[k][1], 1))[:3] for k in links) / M
        return M, c


# ---------------------------------------------------------------- DH check
# URDF reference points for the DH frames: (link, local offset in mm)
URDF_POINTS = {
    "l_leg": [("l_knee_link", (0, 0, 0)), ("l_ankle_roll_link", (0, 0, 0)), ("l_ankle_roll_link", (0, 0, -25))],
    "r_leg": [("r_knee_link", (0, 0, 0)), ("r_ankle_roll_link", (0, 0, 0)), ("r_ankle_roll_link", (0, 0, -25))],
    "l_arm": [("l_elbow_link", (0, 0, 0)), ("l_elbow_link", (0, 0, -44))],
    "r_arm": [("r_elbow_link", (0, 0, 0)), ("r_elbow_link", (0, 0, -44))],
    "head":  [("head_pitch_link", (0, 0, 0)), ("head_pitch_link", (28, 0, 24))],
}
# which DH frame index each URDF point corresponds to (frame list from fk: 0 = base->frame0)
DH_INDEX = {"l_leg": [3, 4, 6], "r_leg": [3, 4, 6], "l_arm": [2, 3], "r_arm": [2, 3], "head": [2, 3]}


def verify_dh(u, n=300, seed=1):
    rng = np.random.default_rng(seed)
    worst = {}
    for _ in range(n):
        q = {j["name"]: rng.uniform(j["lower"], j["upper"]) for j in u.movable}
        X = u.fk(q)
        for ch, refs in URDF_POINTS.items():
            pts, _ = fk(ch, q)
            for (link, off), idx in zip(refs, DH_INDEX[ch]):
                p = (X[link] @ np.append(np.array(off) / 1000, 1))[:3] * 1000
                worst[ch] = max(worst.get(ch, 0), np.linalg.norm(p - pts[idx]))
    return worst


# ---------------------------------------------------------------- dynamics
def link_table(u):
    return [dict(link=k, mass_g=m * 1000, com_mm=" / ".join(f"{v:.1f}" for v in c * 1000),
                 inertia_gmm2=" / ".join(f"{v:.0f}" for v in i * 1e9))      # kg.m2 -> g.mm2
            for k, (m, c, i) in u.links.items() if m > 0]


def hanging_loads(u, X, gdir):
    """Gravity torque (N.m) at every joint from the links beyond it, with the base held: the arm / head case."""
    out = {}
    for j in u.movable:
        M, c = u.mass_com(X, u.subtree(j["child"]))
        p, a = X[j["child"]][:3, 3], X[j["child"]][:3, :3] @ j["axis"]
        out[j["name"]] = abs(np.dot(a, np.cross(c - p, M * G * gdir)))
    return out


def sole_frame(u, X, side):
    """World = stance sole, flat on the floor: re-root the FK there so gravity is -z of the sole."""
    sole = X[f"{side}_sole"]
    return {k: np.linalg.inv(sole) @ T for k, T in X.items()}


def standing_loads(u, q, feet):
    """Static torque at the stance-leg joints with `feet` = {side: share of the weight} on the floor.
    Ground reaction: vertical, share * W, under the robot CoM (x) and that sole (y). Joints outside the
    stance legs carry their own subtree as in hanging_loads."""
    Xw = sole_frame(u, u.fk(q), next(iter(feet)))
    allk = list(u.links)
    W, com = u.mass_com(Xw, allk)
    gz = np.array([0, 0, -1.0])
    out = hanging_loads(u, Xw, gz)
    for side, share in feet.items():
        cop = np.array([com[0], Xw[f"{side}_sole"][1, 3], Xw[f"{side}_sole"][2, 3]])
        for j in u.movable:
            if not j["name"].startswith(f"{side}_") or j["name"].split("_", 1)[1] not in LEG_JOINTS:
                continue
            p, a = Xw[j["child"]][:3, 3], Xw[j["child"]][:3, :3] @ j["axis"]
            foot_side = u.subtree(j["child"])                 # links between this joint and the floor
            Mf, cf = u.mass_com(Xw, foot_side)
            tau = np.cross(cop - p, share * W * G * -gz) + np.cross(cf - p, Mf * G * gz)
            out[j["name"]] = abs(np.dot(a, tau))
    return out


LEG_JOINTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")


def static_cases(u):
    """Worst static torque per joint (N.m) and the case it comes from."""
    hp, kn, ap = gait.leg_ik(0, 0.070)                                 # squat: hip-to-ankle 70 mm
    single = gait.step_pose(0.5, "r", 0, 0, (0, 0), (0, 0))           # on the left foot, right foot lifted
    single.update(l_elbow=-0.4, r_elbow=-0.4)
    hd = u.joints["head_pitch"]["upper"]                              # head fully down, at its joint limit
    squat = {f"{s}_{j}": a for s in "lr" for j, a in zip(("hip_pitch", "knee", "ankle_pitch"), (hp, kn, ap))}
    cases = {
        "arms forward": hanging_loads(u, u.fk(dict(l_shoulder_pitch=-math.pi / 2, r_shoulder_pitch=-math.pi / 2)),
                                      np.array([0, 0, -1.0])),
        "arms out to the side": hanging_loads(u, u.fk(dict(l_shoulder_roll=math.pi / 2,
                                                           r_shoulder_roll=-math.pi / 2)), np.array([0, 0, -1.0])),
        f"head down {math.degrees(hd):.0f} deg": hanging_loads(u, u.fk(dict(head_pitch=hd)), np.array([0, 0, -1.0])),
        "one foot (left)": standing_loads(u, single, {"l": 1.0}),
        f"squat, knee {math.degrees(kn):.0f} deg": standing_loads(u, squat, {"l": 0.5}),
    }
    out = {}
    for label, loads in cases.items():
        for k, v in loads.items():
            if v > out.get(k, (0, ""))[0]:
                out[k] = (v, label)
    return out


def planned_motion(dt=0.002):
    """Peak speed and acceleration of every joint along the commanded gait (kinematics only)."""
    ts = np.arange(0, gait.T_END, dt)
    Q = np.array([[gait.targets(t).get(j, 0.0) for j in gait.JOINTS] for t in ts])
    v = np.gradient(Q, dt, axis=0)
    a = np.gradient(v, dt, axis=0)
    return {j: (np.abs(v[:, i]).max(), np.abs(a[:, i]).max()) for i, j in enumerate(gait.JOINTS)}


def mj_loads():
    """Gait torques measured in MuJoCo by sim/mujoco/check_mujoco.py (p99 and RMS, N.m), if that has been run."""
    f = OUT / "mj_servo_loads.csv"
    if not f.exists():
        return {}
    with open(f, encoding="utf-8") as fh:
        return {r["joint"]: (float(r["p99_Nm"]), float(r["rms_Nm"])) for r in csv.DictReader(fh)}


# ---------------------------------------------------------------- main
def main():
    u = Urdf(HERE / "humanoid.urdf")
    OUT.mkdir(exist_ok=True)
    import gen_robot
    servo_of = {b.joint.name: b.joint.servo for b in gen_robot.walk(gen_robot.ROBOT) if b.joint}

    # DH
    worst = verify_dh(u)
    with open(OUT / "dh_table.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["chain", "i", "joint", "theta_i (deg)", "d_i (mm)", "a_i (mm)", "alpha_i (deg)", "servo"])
        for ch, c in CHAINS.items():
            for i, (name, off, dd, a, al) in enumerate(c["dh"], 1):
                th = "q" if off == 0 else f"q {'+' if off > 0 else '-'} {abs(off)}"
                w.writerow([ch, i, name, th, dd, a, al, servo_of[name]])
    print("DH vs URDF, worst position error over 300 random poses (mm):",
          {k: round(float(v), 6) for k, v in worst.items()})

    # links
    with open(OUT / "links_dynamics.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["link", "mass (g)", "CoM x,y,z (mm, link frame)", "Ixx,Iyy,Izz principal (g.mm^2)"])
        for r in link_table(u):
            w.writerow([r["link"], f"{r['mass_g']:.1f}", r["com_mm"], r["inertia_gmm2"]])

    # joints
    stat = static_cases(u)
    motion = planned_motion()
    mj = mj_loads()
    rows = []
    for j in u.movable:
        name = j["name"]
        sv = servo_of[name]
        stall = SERVOS[sv]["stall"]
        rated = stall / 3
        st, st_case = stat.get(name, (0.0, ""))
        p99, rms = mj.get(name, (float("nan"), float("nan")))
        need = max(st, 0.0 if math.isnan(p99) else p99)
        rows.append(dict(
            joint=name, servo=sv, range=f"{math.degrees(j['lower']):.0f}…{math.degrees(j['upper']):.0f}",
            vmax=(math.pi / 3) / SERVOS[sv]["speed"], vpeak=motion[name][0], apeak=motion[name][1],
            stall_kgcm=stall / 0.0981, rated_kgcm=rated / 0.0981, static_kgcm=st / 0.0981, static_case=st_case,
            mj_kgcm=p99 / 0.0981, rms_kgcm=rms / 0.0981, margin=rated / need if need > 0 else float("inf"),
        ))
    with open(OUT / "joints_kin_dyn.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["joint", "servo", "range (deg)", "servo max speed (rad/s)", "gait peak speed (rad/s)",
                    "gait peak accel (rad/s^2)", "stall (kg.cm)", "rated 1/3 (kg.cm)", "static worst (kg.cm)",
                    "static case", "MuJoCo gait p99 (kg.cm)", "MuJoCo gait RMS (kg.cm)", "margin rated/need"])
        for r in rows:
            w.writerow([r["joint"], r["servo"], r["range"], f"{r['vmax']:.1f}", f"{r['vpeak']:.2f}",
                        f"{r['apeak']:.0f}", f"{r['stall_kgcm']:.2f}", f"{r['rated_kgcm']:.2f}",
                        f"{r['static_kgcm']:.2f}", r["static_case"], f"{r['mj_kgcm']:.2f}",
                        f"{r['rms_kgcm']:.2f}", f"{r['margin']:.2f}"])
    print(f"wrote {OUT}/dh_table.csv, links_dynamics.csv, joints_kin_dyn.csv"
          + ("" if mj else "  (no MuJoCo loads yet: run sim/mujoco/check_mujoco.py)"))
    print(f"total mass {sum(m for m, _, _ in u.links.values()) * 1000:.0f} g")
    for r in rows:
        print(f"{r['joint']:18s} {r['servo']:6s} static {r['static_kgcm']:.2f} mj {r['mj_kgcm']:.2f} "
              f"rated {r['rated_kgcm']:.2f} margin {r['margin']:.2f}  ({r['static_case']})")


if __name__ == "__main__":
    main()
