"""The digital twin from the CAD: bazan_cad.urdf, generated, never edited by hand.

    python gen_urdf.py            # writes software/ros2/src/humanoid_description/urdf/bazan_cad.urdf (about 2 min)
    python gen_urdf.py --check    # exit 1 if the committed file is not what the CAD gives today
    python gen_urdf.py --validate # also load it in Pinocchio and MuJoCo and print the sanity report

Same link and joint names, joint axis signs and joint limits as bazan.urdf.xacro (the firmware, the controllers and
the RL policy use those), but everything the CAD knows comes from the CAD:

    joint origins   a point on each CAD joint axis: the point closest to the next joint axis down the chain (or to
                    the child link's centre of mass at the end of a chain), so every axis passes exactly where the
                    printed parts put it
    inertials       every component of audit.weigh() (printed parts with their print recipe, servos, boards,
                    battery, shell, wiring) integrated with its own solid: mass, centre of mass and the full inertia
                    tensor about the link CoM (parallel axis theorem), at the zero pose
    frames          l_sole / r_sole on the floor under the ankle, imu_link where the MPU6050 sits on the A+ board

Units: the CAD is in mm and g, the URDF in m and kg. Every link frame is parallel to the robot frame at the zero
pose (x forward, y left, z up, REP-103), base_link at the middle of the two hip points H as before. Collision and
visual geometry are the link's bounding boxes; the visual meshes stay with gen_robot.py.
"""
import sys
from pathlib import Path

import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

import audit as A
import leg as LEG
import pose as P
import robot as R
import shell as SH

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "software" / "ros2" / "src" / "humanoid_description" / "urdf" / "bazan_cad.urdf"
XACRO = A.XACRO

# URDF link of every CAD link (robot.link_of names), and the URDF joints in chain order with their CAD joint
LINK = {"torso": "base_link", "neck_link": "neck_yaw_link", "head_frame": "head_pitch_link"}
for _s, _u in (("L", "l"), ("R", "r")):
    LINK.update({f"{_s}.leg.yaw_bracket": f"{_u}_hip_yaw_link", f"{_s}.leg.hip_block": f"{_u}_hip_roll_link",
                 f"{_s}.leg.thigh": f"{_u}_hip_pitch_link", f"{_s}.leg.shin": f"{_u}_knee_link",
                 f"{_s}.leg.ankle_block": f"{_u}_ankle_pitch_link", f"{_s}.leg.foot": f"{_u}_ankle_roll_link",
                 f"{_s}.shoulder_link": f"{_u}_shoulder_pitch_link", f"{_s}.upper_arm": f"{_u}_shoulder_roll_link",
                 f"{_s}.forearm": f"{_u}_elbow_link"})
LEG_CHAIN = ["hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll"]
ARM_CHAIN = ["shoulder_pitch", "shoulder_roll", "elbow"]
HEAD_CHAIN = ["neck_yaw", "head_pitch"]


def inertia(solid):
    """(volume mm3, centre mm, inertia about the centre per unit density, mm5) of an OCC solid."""
    pr = GProp_GProps()
    BRepGProp.VolumeProperties_s(solid.wrapped, pr)
    c = pr.CentreOfMass()
    m = pr.MatrixOfInertia()
    J_ = np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    return pr.Mass(), np.array([c.X(), c.Y(), c.Z()]), J_


def link_inertials(items):
    """{CAD link: (g, CoM mm, inertia about the CoM g.mm2)} from audit.weigh() items."""
    acc = {}
    for name, ln, g, s in items:
        if isinstance(s, np.ndarray):                     # a point mass (wiring allowance)
            c, Ic = s, np.zeros((3, 3))
        else:
            v, c, Iu = inertia(s)
            Ic = Iu * (g / v)                             # uniform density over this component's solid
        acc.setdefault(ln, []).append((g, c, Ic))
    out = {}
    for ln, rows in acc.items():
        g = sum(r[0] for r in rows)
        com = sum(r[0] * r[1] for r in rows) / g
        J_ = np.zeros((3, 3))
        for gi, ci, Ii in rows:
            d = ci - com
            J_ += Ii + gi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        out[ln] = (g, com, J_)
    return out


def closest_on_axis(p, a, q, b=None):
    """Point on the line (p, a) closest to the line (q, b), or to the point q when b is None."""
    a = a / np.linalg.norm(a)
    if b is None:
        return p + a * np.dot(q - p, a)
    b = b / np.linalg.norm(b)
    w = p - q
    aa, bb, ab = 1.0, 1.0, np.dot(a, b)
    d, e = np.dot(a, w), np.dot(b, w)
    den = aa * bb - ab * ab
    if den < 1e-9:                                         # parallel axes: any point; keep the projection of q
        return p + a * np.dot(q - p, a)
    s = (ab * e - bb * d) / den
    return p + s * a


def cad_axes(body, leg):
    """{urdf joint: (point on axis mm, unit axis)} in the robot frame at the zero pose, from the CAD."""
    ax = {}
    for side, sy in (("l", 1), ("r", -1)):
        for jn in LEG_CHAIN:
            p = np.array(leg.joints[jn].origin, float) + [0, LEG.HIP_Y, 0]   # leg frame -> robot frame
            a = np.array(P.AXIS[jn], float)
            ax[f"{side}_{jn}"] = (p * [1, sy, 1], a * [1, sy, 1])          # right side: mirror in y
        for jn, _, _, a in R.ARM_CHAIN:
            p = np.array(body.joints[jn].origin, float)
            ax[f"{side}_{jn}"] = (p * [1, sy, 1], np.array(a, float) * [1, sy, 1])
    for jn, _, _, a in R.HEAD_CHAIN:
        ax[jn] = (np.array(body.joints[jn].origin, float), np.array(a, float))
    return ax


def xacro_joints():
    """{joint: dict(parent, child, axis, lower, upper, effort, velocity)} from bazan.urdf.xacro (kept as is)."""
    import re
    pat = re.compile(r'<joint name="(\w+)" type="revolute"><parent link="(\w+)"/><child link="(\w+)"/>'
                     r'<origin xyz="[^"]+"/><axis xyz="([^"]+)"/><limit lower="([-\d.]+)" upper="([-\d.]+)" '
                     r'effort="([\d.]+)" velocity="([\d.]+)"/>')
    return {m[0]: dict(parent=m[1], child=m[2], axis=np.array(m[3].split(), float), lower=float(m[4]),
                       upper=float(m[5]), effort=float(m[6]), velocity=float(m[7]))
            for m in pat.findall(XACRO.read_text())}


def build():
    sh = SH.BazanShell()
    body, leg = sh.body, sh.leg
    _, _, items = A.weigh(leg, body, sh)
    inert = link_inertials(items)
    cad = cad_axes(body, leg)
    xj = xacro_joints()
    links = {LINK.get(k, k): v for k, v in inert.items()}
    missing = set(LINK.values()) - set(links)
    assert not missing, f"links with no CAD component: {missing}"

    # joint origins in the robot frame: on the CAD axis, closest to the next axis (or the child CoM at a chain end)
    chains = [[f"{s}_{j}" for j in LEG_CHAIN] for s in "lr"] + [[f"{s}_{j}" for j in ARM_CHAIN] for s in "lr"] \
        + [HEAD_CHAIN]
    origin = {}
    for ch in chains:
        for i, jn in enumerate(ch):
            p, a = cad[jn]
            if i + 1 < len(ch):
                q, b = cad[ch[i + 1]]
                origin[jn] = closest_on_axis(p, a, q, b)
            else:
                origin[jn] = closest_on_axis(p, a, links[xj[jn]["child"]][1])
    frame = {"base_link": np.zeros(3)}
    for jn, d in xj.items():
        frame[d["child"]] = origin[jn]
    return dict(body=body, leg=leg, links=links, frame=frame, origin=origin, xj=xj, cad=cad,
                floor=leg.floor, items=items)


def _box(link, m):
    """Bounding box of a link's components (robot frame, mm), for visual/collision."""
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for name, ln, g, s in m["items"]:
        if LINK.get(ln, ln) != link or isinstance(s, np.ndarray):
            continue
        b = s.BoundingBox()
        lo = np.minimum(lo, [b.xmin, b.ymin, b.zmin])
        hi = np.maximum(hi, [b.xmax, b.ymax, b.zmax])
    return lo, hi


def urdf_text(m):
    f = lambda v: " ".join(f"{x:.5f}" for x in v)                       # noqa: E731
    L = ['<?xml version="1.0"?>',
         "<!-- Generated by hardware/mechanical/gen_urdf.py from the CAD (leg.py, body.py, shell.py, audit.weigh)."
         " Do not edit: run python gen_urdf.py. -->",
         '<robot name="bazan">',
         '  <material name="frame"><color rgba="0.88 0.88 0.86 1"/></material>']
    order = ["base_link"] + [d["child"] for d in m["xj"].values()]
    for ln in order:
        g, com, J_ = m["links"][ln]
        o = m["frame"][ln]
        lo, hi = _box(ln, m)
        c, size = (lo + hi) / 2 - o, hi - lo
        Ik = J_ * 1e-9                                                     # g.mm2 -> kg.m2
        L.append(f'  <link name="{ln}">')
        L.append(f'    <inertial><origin xyz="{f((com - o) / 1000)}" rpy="0 0 0"/><mass value="{g / 1000:.5f}"/>'
                 f'<inertia ixx="{Ik[0, 0]:.4e}" iyy="{Ik[1, 1]:.4e}" izz="{Ik[2, 2]:.4e}" ixy="{Ik[0, 1]:.4e}" '
                 f'ixz="{Ik[0, 2]:.4e}" iyz="{Ik[1, 2]:.4e}"/></inertial>')
        L.append(f'    <visual><origin xyz="{f(c / 1000)}"/><geometry><box size="{f(size / 1000)}"/></geometry>'
                 '<material name="frame"/></visual>')
        L.append(f'    <collision><origin xyz="{f(c / 1000)}"/><geometry><box size="{f(size / 1000)}"/></geometry>'
                 '</collision>')
        L.append("  </link>")
    for jn, d in m["xj"].items():
        rel = (m["frame"][d["child"]] - m["frame"][d["parent"]]) / 1000
        L.append(f'  <joint name="{jn}" type="revolute"><parent link="{d["parent"]}"/><child link="{d["child"]}"/>'
                 f'<origin xyz="{f(rel)}"/><axis xyz="{" ".join(f"{v:g}" for v in d["axis"])}"/>'
                 f'<limit lower="{d["lower"]:.4f}" upper="{d["upper"]:.4f}" effort="{d["effort"]:g}" '
                 f'velocity="{d["velocity"]:g}"/><dynamics damping="0.02" friction="0"/></joint>')
    # REP-120 frames the software uses
    for s in "lr":
        ar = m["frame"][f"{s}_ankle_roll_link"]
        L.append(f'  <link name="{s}_sole"/><joint name="{s}_sole_joint" type="fixed">'
                 f'<parent link="{s}_ankle_roll_link"/><child link="{s}_sole"/>'
                 f'<origin xyz="0 0 {(m["floor"] - ar[2]) / 1000:.5f}" rpy="0 0 0"/></joint>')
    import sensors
    imu = np.array(sensors.get_body_sensors(m["body"])["imu_frame"].pos)          # one source for the IMU frame
    L.append('  <link name="imu_link"/><joint name="imu_link_joint" type="fixed"><parent link="base_link"/>'
             f'<child link="imu_link"/><origin xyz="{f(imu / 1000)}" rpy="0 0 0"/></joint>')
    L.append("</robot>")
    return "\n".join(L) + "\n"


# ------------------------------------------------------------------ validation (the skill's checklist)
def validate(m, path=OUT):
    import mujoco
    import pinocchio as pin
    bad = 0
    print("links: mass, CoM (link frame, mm), principal moments (g.mm2), checks")
    for ln, (g, com, J_) in m["links"].items():
        ev = np.linalg.eigvalsh(J_)
        tri = all(ev[i] + ev[j] >= ev[k] * (1 - 1e-6) for i, j, k in ((0, 1, 2), (1, 2, 0), (0, 2, 1)))
        lo, hi = _box(ln, m)
        e = hi - lo
        # physical bound: all the mass in the corners of the link's bounding box (a solid box gives g/12)
        box_max = g / 4 * np.array([e[1] ** 2 + e[2] ** 2, e[0] ** 2 + e[2] ** 2, e[0] ** 2 + e[1] ** 2]).max()
        com_in = bool(np.all(com >= lo - 0.5) and np.all(com <= hi + 0.5))
        ok = g > 0 and ev.min() > 0 and tri and ev.max() <= box_max * 1.05 and com_in
        bad += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} {ln:22s} {g:7.1f} g  CoM {np.round(com - m['frame'][ln], 1)}  "
              f"I {np.round(ev, 0)}  PD {ev.min() > 0} tri {tri} <=box {ev.max() <= box_max * 1.05} CoM-in {com_in}")
    print("joints: URDF origin vs CAD axis (perpendicular distance, mm)")
    for jn, d in m["xj"].items():
        p, a = m["cad"][jn]
        o = m["frame"][d["child"]]
        v = o - p
        off = np.linalg.norm(v - a * np.dot(v, a) / np.dot(a, a))
        par = abs(abs(np.dot(a, d["axis"])) / np.linalg.norm(a) / np.linalg.norm(d["axis"]) - 1) < 1e-6
        ok = off <= 0.5 and par
        bad += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} {jn:18s} origin {np.round(o, 1)}  off-axis {off:.2f}  "
              f"axis {d['axis']} (kept from the xacro) parallel to the CAD axis: {par}")
    tot = sum(g for g, _, _ in m["links"].values())
    com = sum(g * c for g, c, _ in m["links"].values()) / tot
    print(f"robot: {tot:.0f} g, CoM {np.round(com, 1)} mm ({com[2] - m['floor']:.1f} above the floor), hip point "
          f"{-m['floor']:.1f} above the floor")
    model = pin.buildModelFromUrdf(str(path), pin.JointModelFreeFlyer())     # the robot is not bolted down
    data = model.createData()
    q0 = pin.neutral(model)
    c = pin.centerOfMass(model, data, q0)
    pm = pin.computeTotalMass(model)
    print(f"Pinocchio: {model.njoints - 1} joints, nq {model.nq}, mass {pm * 1000:.0f} g, "
          f"CoM {np.round(c * 1000, 1)} mm")
    if abs(pm * 1000 - tot) > 0.5 or np.linalg.norm(c * 1000 - com) > 0.5:
        bad += 1
        print("  FAIL: Pinocchio disagrees with the CAD sum")
    spec = mujoco.MjSpec.from_file(str(path))
    root = spec.worldbody.first_body()
    root.add_freejoint()                                                    # free base, as on the floor
    mj = spec.compile()
    print(f"MuJoCo: {mj.nbody - 1} bodies, {mj.njnt} joints (1 free), mass {mj.body_mass.sum() * 1000:.0f} g")
    if abs(mj.body_mass.sum() * 1000 - tot) > 0.5:
        bad += 1
        print("  FAIL: MuJoCo disagrees with the CAD sum")
    print("validation:", "0 defects" if not bad else f"{bad} defect(s)")
    return bad


def main():
    m = build()
    txt = urdf_text(m)
    if "--check" in sys.argv:
        if not OUT.exists() or OUT.read_text() != txt:
            print(f"{OUT.relative_to(REPO)} is stale: run python gen_urdf.py")
            return 1
        print("up to date")
    else:
        OUT.write_text(txt)
        print("wrote", OUT.relative_to(REPO))
    if "--validate" in sys.argv:
        return 1 if validate(m) else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
