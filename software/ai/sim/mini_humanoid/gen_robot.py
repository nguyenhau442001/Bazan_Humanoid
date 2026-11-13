"""Generate the URDF (ROS 2, RViz2) and the xacro (RViz2, MuJoCo) for Mico, the 20-DoF mini humanoid.

Single source of truth: every size, mass and joint below comes from the design doc
"Mini Humanoid - Hardware Design". Edit here, re-run, both files update together.

    python gen_robot.py    -> humanoid.urdf, ../../src/humanoid_description/urdf/mico.urdf.xacro

Units in this file: millimetres and grams. Output files use metres and kilograms.
Frame: x forward, y left, z up. Feet soles touch z = 0 in the default pose.
"""
import os
from dataclasses import dataclass, field
from math import pi
from pathlib import Path

# ---------------------------------------------------------------- servos
# Stall torque (N.m) is the hard limit given to the simulator.
# Firmware should keep continuous load below ~1/3 of stall.
SERVOS = {
    # heavy joints: hip roll/pitch, knee, ankle pitch/roll (sold in HCMC, Hshop / icdayroi)
    "MG92B": dict(stall=0.343, speed=0.08, mass=14.0, size=(22.8, 12, 31)),   # TowerPro, 3.5 kg.cm @6V, metal gear
    # medium: hip yaw, shoulder pitch (metal gears survive falls)
    "MG90S": dict(stall=0.216, speed=0.08, mass=13.4, size=(22.8, 12.2, 28.5)),  # 2.2 kg.cm
    # light: shoulder roll, elbow, head
    "SG90": dict(stall=0.177, speed=0.10, mass=9.0, size=(23, 12.2, 29)),      # 1.8 kg.cm @4.8V, plastic gear
    # stronger import-only option, kept for comparison runs (LEG_HEAVY=DS843MG)
    "DS843MG": dict(stall=0.471, speed=0.10, mass=8.5, size=(23, 9, 23)),     # Corona, 4.8 kg.cm @6V
}
LEG_HEAVY = os.environ.get("LEG_HEAVY", "MG92B")
LEG_SERVO = {}
# Feet sit 6 mm outboard of the ankle axis: with them centred the two soles (42 wide, 44 apart)
# rubbed together while walking and turning. 6 mm leaves >= 3.8 mm clearance during the demo.
FOOT_OUT = 0.0
DAMPING = 0.02    # joint damping, N.m.s/rad

# ---------------------------------------------------------------- colours
C = {
    "frame": (0.88, 0.88, 0.86, 1),
    "shell": (0.95, 0.95, 0.93, 1),
    "mg92b": (0.36, 0.56, 0.90, 1),
    "ds843": (0.36, 0.56, 0.90, 1),
    "sg90": (0.45, 0.70, 0.95, 1),
    "mg90s": (0.30, 0.36, 0.52, 1),
    "battery": (0.93, 0.55, 0.42, 1),
    "pcb": (0.36, 0.75, 0.60, 1),
    "lens": (0.10, 0.10, 0.12, 1),
    "hidden": (0, 0, 0, 0),
}


@dataclass
class Geom:
    name: str
    size: tuple          # full box size x, y, z (mm)
    pos: tuple           # centre, relative to body frame (mm)
    mass: float          # g
    color: str


@dataclass
class Joint:
    name: str
    axis: tuple
    range_deg: tuple
    servo: str


@dataclass
class Body:
    name: str
    pos: tuple                       # relative to parent body (mm)
    joint: Joint | None
    geoms: list
    children: list = field(default_factory=list)


HIP_Z = 115      # hip point H: hip yaw, roll and pitch axes all pass through it (mm above floor)
THIGH = SHIN = 45
ANKLE_Z = 25     # ankle point A: ankle pitch and roll axes intersect here


def sv(name, kind, pos, orient="x"):
    """Servo body as a box geom with the real mass and size of that servo type.
    orient: which axis the servo's long side (L) runs along."""
    L, W, H = SERVOS[kind]["size"]
    size = {"x": (L, W, H), "y": (W, L, H), "z": (W, H, L)}[orient]
    color = {"DS843MG": "ds843", "MG90S": "mg90s", "SG90": "sg90", "MG92B": "mg92b"}[kind]
    return Geom(name, size, pos, SERVOS[kind]["mass"], color)


def leg(side):
    """6 DoF: hip yaw -> hip roll -> hip pitch meet at H; ankle pitch -> ankle roll meet at A."""
    s = 1 if side == "l" else -1
    hv = LEG_HEAVY

    def J(j):
        return LEG_SERVO.get(j, hv)

    return Body(f"{side}_hip_yaw", (0, 22 * s, 0),
        Joint(f"{side}_hip_yaw", (0, 0, 1), (-45, 45), "MG90S"),
        [sv(f"{side}_hip_yaw_servo", "MG90S", (0, 0, 22), "z")],                  # upright in the pelvis
        [Body(f"{side}_hip_roll", (0, 0, 0),
            Joint(f"{side}_hip_roll", (1, 0, 0), (-25, 25), J("hip_roll")),
            [sv(f"{side}_hip_roll_servo", J("hip_roll"), (-14, 0, 0), "y")],  # behind H, shaft on x axis
            [Body(f"{side}_hip_pitch", (0, 0, 0),
                Joint(f"{side}_hip_pitch", (0, 1, 0), (-90, 45), J("hip_pitch")),
                [sv(f"{side}_hip_pitch_servo", J("hip_pitch"), (0, 0, -9), "x"),
                 Geom(f"{side}_thigh", (18, 18, THIGH), (0, 0, -THIGH / 2), 11, "shell")],
                [Body(f"{side}_knee", (0, 0, -THIGH),
                    Joint(f"{side}_knee", (0, 1, 0), (0, 120), J("knee")),
                    [sv(f"{side}_knee_servo", J("knee"), (0, 0, 0), "x"),
                     Geom(f"{side}_shin", (18, 18, SHIN), (0, 0, -SHIN / 2), 11, "shell")],
                    [Body(f"{side}_ankle_pitch", (0, 0, -SHIN),
                        Joint(f"{side}_ankle_pitch", (0, 1, 0), (-45, 45), J("ankle_pitch")),
                        [sv(f"{side}_ankle_pitch_servo", J("ankle_pitch"), (0, 0, 11), "x")],
                        [Body(f"{side}_ankle_roll", (0, 0, 0),
                            Joint(f"{side}_ankle_roll", (1, 0, 0), (-30, 30), J("ankle_roll")),
                            [sv(f"{side}_ankle_roll_servo", J("ankle_roll"), (-14, 0, -2), "y"),
                             Geom(f"{side}_foot", (76, 38, 8), (0, 0, -(ANKLE_Z - 4)), 12,
                                  "frame")])])])])])])


def arm(side):
    s = 1 if side == "l" else -1
    roll_range = (-10, 160) if s > 0 else (-160, 10)
    return Body(f"{side}_shoulder_pitch", (0, 50 * s, 97),
        Joint(f"{side}_shoulder_pitch", (0, 1, 0), (-180, 60), "MG90S"),
        [Geom(f"{side}_shoulder_cap", (18, 14, 28), (0, 5 * s, 0), 1, "shell")],
        [Body(f"{side}_shoulder_roll", (0, 5 * s, 0),
            Joint(f"{side}_shoulder_roll", (1, 0, 0), roll_range, "SG90"),
            [sv(f"{side}_shoulder_roll_servo", "SG90", (0, 0, -6), "z"),
             Geom(f"{side}_upper_arm", (16, 16, 36), (0, 0, -30), 3, "shell")],
            [Body(f"{side}_elbow", (0, 0, -50),
                Joint(f"{side}_elbow", (0, 1, 0), (-130, 0), "SG90"),
                [sv(f"{side}_elbow_servo", "SG90", (0, 0, 0), "x"),
                 Geom(f"{side}_forearm", (14, 14, 38), (0, 0, -19), 2, "shell"),
                 Geom(f"{side}_hand", (14, 14, 14), (0, 0, -44), 1.5, "shell")])])])


HEAD = dict(size=(62, 74, 64), pos=(2, 0, 24), mass=14)   # "A" shell: big rounded helmet, ~1/4 of height

# Base frame sits at the hip point H. Everything above is measured from there.
ROBOT = Body("base", (0, 0, HIP_Z), None, [
    Geom("pelvis", (40, 68, 36), (0, 0, 21), 10, "frame"),              # holds both hip-yaw servos
    Geom("torso_shell", (46, 74, 72), (0, 0, 74), 20, "shell"),
    Geom("battery_2s", (30, 56, 19), (0, 0, 49.5), 57, "battery"),     # flat, centred, just above the pelvis
    Geom("electronics", (25, 62, 25), (0, 0, 75), 49, "pcb"),
    Geom("ubec", (8, 26, 46), (-17, 0, 82), 15, "pcb"),
    sv("l_shoulder_pitch_servo", "MG90S", (0, 33.5, 96), "y"),
    sv("r_shoulder_pitch_servo", "MG90S", (0, -33.5, 96), "y"),
    sv("neck_yaw_servo", "SG90", (0, 0, 98), "z"),
    Geom("wiring", (30, 50, 40), (0, 0, 58), 30, "hidden"),
], [
    leg("l"), leg("r"), arm("l"), arm("r"),
    Body("neck_yaw", (0, 0, 110),
        Joint("neck_yaw", (0, 0, 1), (-90, 90), "SG90"),
        [Geom("neck", (16, 16, 10), (0, 0, 5), 4, "frame"),
         sv("head_pitch_servo", "SG90", (0, 0, 22), "y")],
        [Body("head_pitch", (0, 0, 22),
            # 32 deg down keeps a 2 mm shell gap at any neck yaw (docs/design/blender/neck_check.py)
            Joint("head_pitch", (0, 1, 0), (-30, 32), "SG90"),
            [Geom("head_shell", HEAD["size"], HEAD["pos"], HEAD["mass"], "shell"),
             Geom("visor", (2, 58, 26), (34, 0, 24), 0, "lens"),
             Geom("camera", (6, 25, 24), (28, 0, 24), 6, "pcb")])]),
])


# ---------------------------------------------------------------- sensor and tool frames
# (name, parent body, xyz mm in the parent body frame, rpy rad). Every body frame is parallel to base_link
# at the zero pose (REP-103: x forward, y left, z up), so these are also the mounting directions.
CAM_TILT = 15 * pi / 180   # camera pitched down so the floor is visible from ~0.35 m
TOF_TILT = 20 * pi / 180   # ToF pitched down so its beam meets the floor at 0.46-2.2 m
FRAMES = [
    ("imu_link", "head_pitch", (2, 0, 48), (0, 0, 0)),             # MPU9250 9-DOF IMU on head crown (no hip EMF)
    ("camera_link", "head_pitch", (31, 0, 42), (0, CAM_TILT, 0)),
    ("tof_link", "head_pitch", (31, 8, 45), (0, TOF_TILT, 0)),
    ("l_mic_link", "head_pitch", (2, 38.5, 24), (0, 0, pi / 2)),   # mic faces +y (out of the left ear)
    ("r_mic_link", "head_pitch", (2, -38.5, 24), (0, 0, -pi / 2)),
    ("l_sole", "l_ankle_roll", (0, 0, -ANKLE_Z), (0, 0, 0)),       # REP-120: on the sole, under the ankle axis
    ("r_sole", "r_ankle_roll", (0, 0, -ANKLE_Z), (0, 0, 0)),
    ("l_gripper", "l_elbow", (0, 0, -44), (0, 0, 0)),
    ("r_gripper", "r_elbow", (0, 0, -44), (0, 0, 0)),
]
# camera_optical_frame (z forward, x right, y down) hangs off camera_link in the URDF only.
OPTICAL_RPY = (-pi / 2, 0, -pi / 2)


# ---------------------------------------------------------------- helpers
def m(v):
    return " ".join(f"{x / 1000:.4f}" for x in v)


def rgba(c):
    return " ".join(f"{x:g}" for x in C[c])


def walk(body):
    yield body
    for c in body.children:
        yield from walk(c)


# ---------------------------------------------------------------- Blender shell (docs/design/blender)
# Mesh visuals and mass properties of the Mico shell, used by the xacro (look:=mico, mass:=blender).
_REPO = Path(__file__).resolve().parent
while _REPO.parent != _REPO and not (_REPO / ".git").exists() and not (_REPO / "AGENTS.md").exists():
    _REPO = _REPO.parent
BLENDER = _REPO / "design" / "blender"
MESHES = _REPO / "software" / "ros2" / "src" / "humanoid_description" / "meshes" / "visual" / "mico"


def _load_json(path):
    import json
    return json.loads(path.read_text()) if path.exists() else None


SHELL_MESHES = _load_json(MESHES / "manifest.json")          # python docs/design/blender/mico_meshes.py
SHELL_MASS = _load_json(BLENDER / "mass_report.json")        # python docs/design/blender/mico_mass.py


def _inertial_xml(M, com, inertia):
    """URDF inertial with a full tensor (SI units)."""
    ixx, iyy, izz, ixy, ixz, iyz = inertia
    return (f'<inertial><origin xyz="{" ".join(f"{v:.4f}" for v in com)}"/><mass value="{M:.4f}"/>'
            f'<inertia ixx="{ixx:.3e}" iyy="{iyy:.3e}" izz="{izz:.3e}" '
            f'ixy="{ixy:.3e}" ixz="{ixz:.3e}" iyz="{iyz:.3e}"/></inertial>')


# ---------------------------------------------------------------- URDF
def box_inertia(mass, size):
    x, y, z = size
    return (mass / 12 * (y * y + z * z), mass / 12 * (x * x + z * z), mass / 12 * (x * x + y * y))


def link_inertial(geoms):
    """Combine box geoms into one inertial (SI units), parallel-axis about the link CoM."""
    gs = [(g.mass / 1000, [s / 1000 for s in g.size], [p / 1000 for p in g.pos]) for g in geoms if g.mass > 0]
    M = sum(g[0] for g in gs) or 1e-4
    com = [sum(g[0] * g[2][i] for g in gs) / M for i in range(3)] if gs else [0, 0, 0]
    ixx = iyy = izz = 0.0
    for mass, size, pos in gs:
        a, b, c = box_inertia(mass, size)
        dx, dy, dz = (pos[i] - com[i] for i in range(3))
        ixx += a + mass * (dy * dy + dz * dz)
        iyy += b + mass * (dx * dx + dz * dz)
        izz += c + mass * (dx * dx + dy * dy)
    return M, com, (max(ixx, 1e-8), max(iyy, 1e-8), max(izz, 1e-8))


def _mesh_visual(v, indent):
    return (f'{indent}<visual name="{v["file"][:-4]}"><geometry><mesh filename="package://'
            f'humanoid_description/meshes/visual/mico/{v["file"]}"/></geometry>'
            f'<material name="mico_{v["look"]}"/></visual>')


def urdf(xacro=False):
    """xacro=True: wrap visuals in look:=mico|boxes and inertials in mass:=sim|blender."""
    out = ['<?xml version="1.0"?>',
           '<!-- Generated by gen_robot.py — edit that file, not this one. -->',
           '<robot name="mini_humanoid">',
           '  <!-- Frames follow REP-103 (x forward, y left, z up) and REP-120 names (imu_link, l_sole, ...).',
           '       base_link sits at the hip point H, 115 mm above the floor in the standing pose. -->']
    mats = set()
    for b in walk(ROBOT):
        for g in b.geoms:
            mats.add(g.color)
    for c in sorted(mats):
        out.append(f'  <material name="{c}"><color rgba="{rgba(c)}"/></material>')
    if xacro and SHELL_MESHES:
        for k, v in SHELL_MESHES["colours"].items():
            out.append(f'  <material name="mico_{k}"><color rgba="{" ".join(f"{x:g}" for x in v)}"/></material>')

    # RViz2 (Jazzy) paints every <visual> of a link with the material of the first one, so each extra colour of the
    # shell goes on its own massless child link (fixed joint, zero origin): RViz2 then shows the MuJoCo colours.
    colour_links = []
    for b in walk(ROBOT):
        link = "base_link" if b.joint is None else b.name + "_link"
        M, com, inertia = link_inertial(b.geoms)
        out.append(f'  <link name="{link}">')
        sim_inertial = (f'<inertial><origin xyz="{" ".join(f"{v:.4f}" for v in com)}"/>'
                        f'<mass value="{M:.4f}"/>'
                        f'<inertia ixx="{inertia[0]:.3e}" iyy="{inertia[1]:.3e}" izz="{inertia[2]:.3e}" '
                        f'ixy="0" ixz="0" iyz="0"/></inertial>')
        shell = SHELL_MASS["links"].get(b.name) if (xacro and SHELL_MASS) else None
        if shell:
            t = shell["I"]
            shell_inertial = _inertial_xml(shell["mass"] / 1000, [c / 1000 for c in shell["com"]],
                                           [t[0][0] * 1e-9, t[1][1] * 1e-9, t[2][2] * 1e-9,
                                            t[0][1] * 1e-9, t[0][2] * 1e-9, t[1][2] * 1e-9])
            out.append(f"    <xacro:if value=\"${{'$(arg mass)' == 'sim'}}\">{sim_inertial}</xacro:if>")
            out.append(f"    <xacro:if value=\"${{'$(arg mass)' == 'blender'}}\">{shell_inertial}</xacro:if>")
        else:
            out.append(f'    {sim_inertial}')
        meshes = SHELL_MESHES["links"].get(b.name, []) if (xacro and SHELL_MESHES) else []
        if meshes:
            looks = {}
            for v in meshes:
                looks.setdefault(v["look"], []).append(v)
            first, *rest = looks
            out.append("    <xacro:if value=\"${'$(arg look)' == 'mico'}\">")
            out += [_mesh_visual(v, "      ") for v in looks[first]]
            out.append("    </xacro:if>")
            colour_links += [(link, look, looks[look]) for look in rest]
            out.append("    <xacro:if value=\"${'$(arg look)' == 'boxes'}\">")
        boxes = [g for g in b.geoms if g.color != "hidden"]
        vis = [f'    <visual name="{g.name}"><origin xyz="{m(g.pos)}"/>'
               f'<geometry><box size="{m(g.size)}"/></geometry><material name="{g.color}"/></visual>' for g in boxes]
        col = [f'    <collision name="{g.name}_collision"><origin xyz="{m(g.pos)}"/>'
               f'<geometry><box size="{m(g.size)}"/></geometry></collision>' for g in boxes]
        if meshes:
            out += vis + ["    </xacro:if>"] + col
        else:
            if xacro and boxes:
                out += ["    <xacro:if value=\"${'$(arg look)' == 'boxes'}\">"] + vis + ["    </xacro:if>"] + col
            else:
                out += [line for pair in zip(vis, col) for line in pair]
        out.append("  </link>")
    for b in walk(ROBOT):
        parent = "base_link" if b.joint is None else b.name + "_link"
        for c in b.children:
            j = c.joint
            lo, hi = (v * pi / 180 for v in j.range_deg)
            sv = SERVOS[j.servo]
            vel = (pi / 3) / sv["speed"]
            out.append(f'  <joint name="{j.name}" type="revolute">'
                       f'<parent link="{parent}"/><child link="{c.name}_link"/>'
                       f'<origin xyz="{m(c.pos)}"/><axis xyz="{" ".join(map(str, j.axis))}"/>'
                       f'<limit lower="{lo:.4f}" upper="{hi:.4f}" effort="{sv["stall"]}" '
                       f'velocity="{vel:.2f}"/><dynamics damping="{DAMPING}" friction="0"/></joint>')
    if colour_links:
        out.append("  <!-- one colour per link for RViz2: extra shell colours on massless fixed child links -->")
        out.append("  <xacro:if value=\"${'$(arg look)' == 'mico'}\">")
        for parent, look, vs in colour_links:
            name = f"{parent}__{look}"
            out.append(f'    <link name="{name}">')
            out += [_mesh_visual(v, "      ") for v in vs]
            out.append("    </link>")
            out.append(f'    <joint name="{name}_joint" type="fixed"><parent link="{parent}"/><child link="{name}"/>'
                       f'<origin xyz="0 0 0" rpy="0 0 0"/></joint>')
        out.append("  </xacro:if>")
    # sensor and tool frames: massless links on fixed joints
    for name, parent, xyz, rpy in FRAMES:
        plink = "base_link" if parent == "base" else parent + "_link"
        out.append(f'  <link name="{name}"/>')
        out.append(f'  <joint name="{name}_joint" type="fixed"><parent link="{plink}"/><child link="{name}"/>'
                   f'<origin xyz="{m(xyz)}" rpy="{" ".join(f"{a:.6f}" for a in rpy)}"/></joint>')
    out.append('  <link name="camera_optical_frame"/>')
    out.append('  <joint name="camera_optical_joint" type="fixed"><parent link="camera_link"/>'
               '<child link="camera_optical_frame"/>'
               f'<origin xyz="0 0 0" rpy="{" ".join(f"{a:.6f}" for a in OPTICAL_RPY)}"/></joint>')
    out.append("</robot>")
    return "\n".join(out) + "\n"


# ---------------------------------------------------------------- xacro for RViz2 and MuJoCo
def mico_xacro():
    body = urdf(xacro=True).splitlines()[3:-1]
    return f"""<?xml version="1.0"?>
<!-- Generated by sim/mini_humanoid/gen_robot.py — edit that file, not this one.
     Mico for RViz2 and MuJoCo. Same links, joints and frames as humanoid.urdf. -->
<robot name="mico" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- look:=mico draws the Blender shell (meshes/visual/mico), look:=boxes the servo / battery boxes.
       mass:=sim keeps the masses the gait was tuned on, mass:=blender uses the shell mass budget.
       Collisions are always the boxes. -->
  <xacro:arg name="look" default="mico"/>
  <xacro:arg name="mass" default="sim"/>
{chr(10).join(body)}
</robot>
"""


if __name__ == "__main__":
    here = Path(__file__).parent
    (here / "humanoid.urdf").write_text(urdf(), encoding="utf-8")
    desc = _REPO / "software" / "ros2" / "src" / "humanoid_description" / "urdf"
    if desc.is_dir():
        (desc / "mico.urdf.xacro").write_text(mico_xacro(), encoding="utf-8")
    joints = [b.joint for b in walk(ROBOT) if b.joint]
    mass = sum(g.mass for b in walk(ROBOT) for g in b.geoms)
    print(f"wrote humanoid.urdf + mico.urdf.xacro: {len(joints)} joints, {mass:.0f} g")
