"""MuJoCo physics for Bazan, shared by the ROS 2 bridge (bazan_mujoco_bridge.py) and sim/mujoco (no ROS).

The robot is bazan_cad.urdf, generated from the CAD by hardware/mechanical/gen_urdf.py: cad_mjcf() builds it.
build_mjcf() turns any such URDF into a MuJoCo model:
  * the same links, joints, limits, damping and inertials; the link boxes collide (group 0)
  * a free base, a floor (group 2), 1 ms steps
  * one position servo per joint: torque = kp (target - q) - kv dq, clipped to the URDF effort limit (stall torque)
  * armature for the servo gearbox, which also keeps the tiny link inertias stable
  * contacts: robot <-> floor, left leg <-> right leg
  * imu site on imu_link: accelerometer, gyro, orientation; head_camera on camera_link
  * foot_forces() and tof_range(): the FSR402 pads and the VL53L0X, as the policy sees them (bazan_policy_io)

BazanSim steps the model and follows trajectories like joint_trajectory_controller does (linear interpolation
on sim time, the zero stamp means "start now", hold the last point).
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
try:
    import bazan_gait as gait
except ImportError:
    import mico_gait as gait  # noqa: E402

JOINTS = gait.JOINTS
CAD_URDF = gait.URDF
ROBOT, FLOOR, LEFT, RIGHT = 1, 2, 4, 8          # contact bits
LEG_LINKS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll")

# servo model; MG92B / MG90S / SG90 stall torques come from the URDF effort limits
KP = gait.SERVO_KP       # N m / rad: the gait's sag feed-forward assumes the same stiffness
KV = 0.03                # N m s / rad
ARMATURE = 2e-5          # kg m^2 reflected rotor inertia
TIMESTEP = 0.001


def build_mjcf(urdf_text, resolve_mesh, kp=KP, kv=KV, armature=ARMATURE, timestep=TIMESTEP, offscreen=960,
               modelfiledir=None):
    """MJCF string for Mico. resolve_mesh(filename) maps a URDF mesh filename (package://...) to a path;
    relative paths are relative to modelfiledir (the folder the MJCF will be saved in)."""
    root = ET.fromstring(urdf_text)
    # colour links (<link>__<colour>, fixed, zero origin, massless) exist only for RViz2: their visuals go back
    # on the parent link, so the MuJoCo bodies stay the 31 links of the robot
    links = {ln.get("name"): ln for ln in root.findall("link")}
    for j in root.findall("joint"):
        child = j.find("child").get("link")
        if "__" in child and j.get("type") == "fixed":
            parent = links[j.find("parent").get("link")]
            for v in links[child].findall("visual"):
                parent.append(v)
            root.remove(links.pop(child))
            root.remove(j)
    effort = {}
    for j in root.findall("joint"):
        lim = j.find("limit")
        if j.get("type") == "revolute" and lim is not None:
            effort[j.get("name")] = float(lim.get("effort"))
    for m in root.iter("mesh"):
        m.set("filename", resolve_mesh(m.get("filename")))
    mj = ET.SubElement(root, "mujoco")
    ET.SubElement(mj, "compiler", discardvisual="false", strippath="false", fusestatic="false",
                  balanceinertia="true")

    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
    if modelfiledir:
        spec.modelfiledir = str(modelfiledir)
    spec.option.timestep = timestep
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.visual.global_.offwidth = spec.visual.global_.offheight = offscreen
    spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
    spec.body("base_link").add_freejoint(name="floating_base")

    world = spec.worldbody
    world.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.01], group=2,
                   rgba=[0.25, 0.3, 0.38, 1], contype=FLOOR, conaffinity=ROBOT | LEFT | RIGHT)
    world.add_light(pos=[0.3, -0.3, 1.0], dir=[-0.3, 0.3, -1], diffuse=[0.8, 0.8, 0.8])
    world.add_light(pos=[-0.4, 0.4, 0.8], dir=[0.4, -0.4, -1], diffuse=[0.4, 0.4, 0.4])

    for g in spec.geoms:
        if g.name == "floor":
            continue
        side = g.parent.name[0] if g.parent.name[2:-5] in LEG_LINKS else None
        if g.group != 0:                                   # visual
            g.contype = g.conaffinity = 0
        elif side:                                         # a leg meets the floor and the other leg
            g.contype, g.conaffinity = (LEFT, FLOOR | RIGHT) if side == "l" else (RIGHT, FLOOR | LEFT)
        else:
            g.contype, g.conaffinity = ROBOT, FLOOR

    for name in JOINTS:
        spec.joint(name).armature = armature
        f = effort[name]
        lo, hi = spec.joint(name).range
        spec.add_actuator(name=name, target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                          gaintype=mujoco.mjtGain.mjGAIN_FIXED, gainprm=[kp] + [0] * 9,
                          biastype=mujoco.mjtBias.mjBIAS_AFFINE, biasprm=[0, -kp, -kv] + [0] * 7,
                          ctrllimited=True, ctrlrange=[lo, hi], forcelimited=True, forcerange=[-f, f])

    spec.body("imu_link").add_site(name="imu")
    spec.add_sensor(name="imu_acc", type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_gyro", type=mujoco.mjtSensor.mjSENS_GYRO, objtype=mujoco.mjtObj.mjOBJ_SITE,
                    objname="imu")
    spec.add_sensor(name="imu_quat", type=mujoco.mjtSensor.mjSENS_FRAMEQUAT, objtype=mujoco.mjtObj.mjOBJ_SITE,
                    objname="imu")

    cam_link = spec.body("camera_link")
    if cam_link:
        cam = cam_link.add_camera()
        cam.name = "head_camera"
        cam.pos = [0, 0, 0]
        cam.quat = [0.5, -0.5, 0.5, 0.5]
        cam.fovy = 60
    spec.compile()
    return spec.to_xml()


def cad_mjcf(kp=KP, kv=KV, armature=ARMATURE, timestep=TIMESTEP):
    """MJCF string of the robot: bazan_cad.urdf, the twin generated from the CAD."""
    return build_mjcf(CAD_URDF.read_text(), lambda f: f, kp, kv, armature, timestep)


def foot_forces(model, data):
    """Normal contact force (N) under the left heel, left toe, right heel, right toe: the four FSR402 pads. A
    contact counts as toe when it is ahead of the ankle in the sole's own frame."""
    out = np.zeros(4)
    feet = {model.body(f"{s}_ankle_roll_link").id: i for i, s in enumerate("lr")}
    soles = [model.body(f"{s}_sole").id for s in "lr"]
    f6 = np.zeros(6)
    for k in range(data.ncon):
        c = data.contact[k]
        for g in (c.geom1, c.geom2):
            side = feet.get(int(model.geom_bodyid[g]))
            if side is None:
                continue
            mujoco.mj_contactForce(model, data, k, f6)
            sole = soles[side]
            ahead = (data.xmat[sole].reshape(3, 3).T @ (c.pos - data.xpos[sole]))[0] >= 0.0
            out[2 * side + int(ahead)] += abs(f6[0])
    return out


def tof_range(model, data, max_range=2.0):
    """VL53L0X distance (m) along tof_link's x axis, ignoring the head; max_range when nothing is hit."""
    tof = model.body("tof_link").id
    geom = np.zeros(1, dtype=np.int32)
    d = mujoco.mj_ray(model, data, data.xpos[tof], data.xmat[tof].reshape(3, 3)[:, 0], None, 1,
                      model.body("head_pitch_link").id, geom)
    return float(d) if 0.0 <= d <= max_range else max_range


class Trajectory:
    """One JointTrajectory: point times (s from `start`, sim time) and positions (n x len(JOINTS)),
    received at sim time `received`."""

    def __init__(self, start, times, positions, received):
        self.t = start + np.asarray(times, float)
        self.q = np.asarray(positions, float)
        self.start_time = received

    def sample(self, t, q_from):
        """Target at sim time t. Before the first point, ramp from the pose held when the goal arrived."""
        if t >= self.t[-1]:
            return self.q[-1]
        if t <= self.t[0] or len(self.t) == 1:
            if self.t[0] <= self.start_time:
                return self.q[0]
            w = (t - self.start_time) / (self.t[0] - self.start_time)
            return q_from + (self.q[0] - q_from) * min(max(w, 0.0), 1.0)
        i = int(np.searchsorted(self.t, t)) - 1
        w = (t - self.t[i]) / (self.t[i + 1] - self.t[i])
        return self.q[i] + (self.q[i + 1] - self.q[i]) * w


class BazanSim:
    def __init__(self, model):
        """model: MJCF string from build_mjcf, a path to a saved MJCF, or an MjModel."""
        if isinstance(model, mujoco.MjModel):
            self.model = model
        elif str(model).lstrip().startswith("<"):
            self.model = mujoco.MjModel.from_xml_string(model)
        else:
            self.model = mujoco.MjModel.from_xml_path(str(model))
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in JOINTS])
        self.dadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in JOINTS])
        self.act = np.array([m.actuator(j).id for j in JOINTS])
        self.base = m.body("base_link").id
        self.target = np.zeros(len(JOINTS))
        self.traj = None
        self.q_from = self.target.copy()
        sens = {n: (m.sensor_adr[m.sensor(n).id], m.sensor_dim[m.sensor(n).id])
                for n in ("imu_acc", "imu_gyro", "imu_quat")}
        self.sens = {n: slice(a, a + d) for n, (a, d) in sens.items()}

    @property
    def time(self):
        return self.data.time

    def reset(self, pose=None, clearance=0.001):
        """All joints at pose (dict, default 0 = straight legs), lowest sole `clearance` above the floor."""
        mujoco.mj_resetData(self.model, self.data)
        q = np.array([(pose or {}).get(j, 0.0) for j in JOINTS])
        self.data.qpos[self.qadr] = q
        self.data.qpos[3:7] = [1, 0, 0, 0]
        mujoco.mj_kinematics(self.model, self.data)
        soles = [self.model.body(s).id for s in ("l_sole", "r_sole")]
        self.data.qpos[2] = -min(self.data.xpos[s][2] for s in soles) + clearance
        self.target = q.copy()
        self.traj = None
        mujoco.mj_forward(self.model, self.data)

    def set_trajectory(self, joint_names, times, positions, start=None):
        """joint_names may be any order / subset of JOINTS; other joints hold their current target."""
        idx = [JOINTS.index(j) for j in joint_names]
        full = np.tile(self.current_target(), (len(times), 1))
        full[:, idx] = np.asarray(positions, float)
        self.q_from = self.current_target().copy()
        self.traj = Trajectory(self.time if start is None else start, times, full, self.time)

    def set_target(self, q):
        self.traj = None
        self.target = np.asarray(q, float).copy()

    def current_target(self):
        return self.traj.sample(self.time, self.q_from) if self.traj is not None else self.target

    def step(self, n=1):
        for _ in range(n):
            self.data.ctrl[self.act] = self.current_target()
            mujoco.mj_step(self.model, self.data)

    # ------------------------------------------------------------------ state
    def joint_positions(self):
        return self.data.qpos[self.qadr].copy()

    def joint_velocities(self):
        return self.data.qvel[self.dadr].copy()

    def joint_efforts(self):
        return self.data.actuator_force[self.act].copy()

    def base_pose(self):
        """(xyz, quaternion w x y z) of base_link in the world (odom)."""
        return self.data.qpos[:3].copy(), self.data.qpos[3:7].copy()

    def base_twist(self):
        """Linear velocity in the world frame, angular velocity in base_link (MuJoCo free joint convention)."""
        return self.data.qvel[:3].copy(), self.data.qvel[3:6].copy()

    def imu(self):
        """Specific force (m/s^2) and angular rate (rad/s) in imu_link, orientation (w x y z) in the world."""
        s = self.data.sensordata
        return s[self.sens["imu_acc"]].copy(), s[self.sens["imu_gyro"]].copy(), s[self.sens["imu_quat"]].copy()

    def foot_forces(self):
        return foot_forces(self.model, self.data)

    def tof_range(self):
        return tof_range(self.model, self.data)

    def tilt(self):
        """Angle between base_link z and the world z (rad)."""
        zz = self.data.xmat[self.base].reshape(3, 3)[2, 2]
        return math.acos(min(max(zz, -1.0), 1.0))


# Backward-compatibility alias
MicoSim = BazanSim
