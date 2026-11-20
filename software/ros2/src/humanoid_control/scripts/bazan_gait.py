#!/usr/bin/env python3
"""Bazan quasi-static gait, planned on the CAD robot: crouch -> walk 8 steps -> turn left 60 deg -> wave.

Everything physical comes from bazan_cad.urdf (generated from the CAD by hardware/mechanical/gen_urdf.py): leg
geometry, joint axes and every link's mass and centre of mass. Nothing here is typed twice.

Each step: shift the whole-body CoM over the stance foot (both feet down), swing the other foot to its next place
with the CoM held there, pause with both feet down. Joint angles come from inverse kinematics of the pelvis and
both soles; the pelvis is placed so that the CoM of the CAD masses (arms and swing leg included) lands on the
planned point. The pelvis rolls toward the loaded foot (LEAN at full load), which takes the torso's weight off
the stance hip-roll servo. Every command carries the servo sag the static load causes (torque / SERVO_KP), so
the stance leg holds the planned pose.

Sends the whole demo as one trajectory_msgs/JointTrajectory (50 Hz samples) to
/joint_trajectory_controller/joint_trajectory. The MuJoCo bridge interpolates it on sim time.

    ros2 run humanoid_control bazan_gait.py
    ros2 run humanoid_control bazan_gait.py --ros-args -p loop:=true
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

JOINTS = ["l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle_pitch", "l_ankle_roll",
          "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle_pitch", "r_ankle_roll",
          "l_shoulder_pitch", "l_shoulder_roll", "l_elbow", "r_shoulder_pitch", "r_shoulder_roll", "r_elbow",
          "neck_yaw", "head_pitch"]


def _urdf_path():
    here = Path(__file__).resolve().parent
    p = here.parents[1] / "humanoid_description" / "urdf" / "bazan_cad.urdf"     # source tree
    if p.exists():
        return p
    from ament_index_python.packages import get_package_share_directory           # installed
    return Path(get_package_share_directory("humanoid_description")) / "urdf" / "bazan_cad.urdf"


URDF = _urdf_path()


def _rot(axis, a):
    x, y, z = axis
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def Rz(a):
    return _rot((0.0, 0.0, 1.0), a)


def Rx(a):
    return _rot((1.0, 0.0, 0.0), a)


class Model:
    """Kinematic tree, masses and centres of mass of the URDF (frames parallel at the zero pose, no rpy)."""

    def __init__(self, path=URDF):
        root = ET.parse(path).getroot()
        vec = lambda s: np.array([float(v) for v in s.split()])                      # noqa: E731
        self.mass, self.com = {}, {}
        for ln in root.findall("link"):
            i = ln.find("inertial")
            self.mass[ln.get("name")] = float(i.find("mass").get("value")) if i is not None else 0.0
            self.com[ln.get("name")] = vec(i.find("origin").get("xyz")) if i is not None else np.zeros(3)
        self.joints = {}
        for j in root.findall("joint"):
            ax, lim = j.find("axis"), j.find("limit")
            self.joints[j.get("name")] = dict(
                parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                xyz=vec(j.find("origin").get("xyz")), axis=vec(ax.get("xyz")) if ax is not None else None,
                limit=(float(lim.get("lower")), float(lim.get("upper"))) if lim is not None else None,
                effort=float(lim.get("effort")) if lim is not None else None)
        self.by_child = {d["child"]: n for n, d in self.joints.items()}
        self.order, todo = [], ["base_link"]
        while todo:                                     # parents before children
            ln = todo.pop(0)
            self.order.append(ln)
            todo += [d["child"] for d in self.joints.values() if d["parent"] == ln]
        self.total = sum(self.mass.values())
        self.distal = {ln: self._subtree(ln) for ln in self.order}

    def _subtree(self, link):
        out = [link]
        for ln in out:
            out += [d["child"] for d in self.joints.values() if d["parent"] == ln]
        return set(out)

    def fk(self, q):
        """{link: (R, p)} in the base_link frame for joint angles q (dict, rad)."""
        T = {"base_link": (np.eye(3), np.zeros(3))}
        for ln in self.order[1:]:
            name = self.by_child[ln]
            j = self.joints[name]
            Rp, pp = T[j["parent"]]
            R = Rp if j["axis"] is None else Rp @ _rot(j["axis"], q.get(name, 0.0))
            T[ln] = (R, pp + Rp @ j["xyz"])
        return T

    def com_of(self, T):
        return sum(self.mass[ln] * (T[ln][1] + T[ln][0] @ self.com[ln]) for ln in T) / self.total

    def hold_torques(self, T, Rb, load):
        """Servo torque that holds every joint still under gravity. load: share of the weight on each foot (a
        leg joint carries the rest of the robot in proportion to its foot's share, its own distal links else)."""
        g = np.array([0.0, 0.0, -9.81])
        mc = {ln: self.mass[ln] * (Rb @ (T[ln][1] + T[ln][0] @ self.com[ln])) for ln in T}
        m_all, mc_all = self.total, sum(mc.values())
        out = {}
        for name, j in self.joints.items():
            if j["axis"] is None:
                continue
            R, p = T[j["child"]]
            ax, p = Rb @ R @ j["axis"], Rb @ p
            dl = self.distal[j["child"]]
            m_d, mc_d = sum(self.mass[ln] for ln in dl), sum(mc[ln] for ln in dl)
            hold = -ax @ np.cross(mc_d - m_d * p, g)                       # carry the distal links
            rest = ax @ np.cross((mc_all - mc_d) - (m_all - m_d) * p, g)   # carry the robot on this foot
            w = load.get(name[0], 0.0) if name.split("_")[1] in ("hip", "knee", "ankle") else 0.0
            out[name] = (1 - w) * hold + w * rest
        return out


MODEL = Model()
_J = MODEL.joints
PIVOT = {s: _J[f"{s}_hip_yaw"]["xyz"] for s in "lr"}            # hip yaw and hip roll axes cross here
ROLL_UP = -_J["l_hip_pitch"]["xyz"][2]                         # hip roll axis above the hip pitch axis
L1, L2 = -_J["l_knee"]["xyz"][2], -_J["l_ankle_pitch"]["xyz"][2]   # thigh, shin (hip pitch -> knee -> ankle)
ANKLE_X, ROLL_DOWN = _J["l_ankle_roll"]["xyz"][0], -_J["l_ankle_roll"]["xyz"][2]
SOLE = -_J["l_sole_joint"]["xyz"][2]                            # ankle roll axis above the floor
HIP_Y = PIVOT["l"][1]
LIMIT = {j: _J[j]["limit"] for j in JOINTS}

# ---------------------------------------------------------------- gait parameters
H0 = L1 + L2 - 0.006  # nominal hip-pitch to ankle-pitch height (m): legs bent 6 mm short of straight (v3 0.090)
STEP = 0.056         # stride (m) per step: the longest that keeps the knee p99 under 1/3 stall (leg v4, check_mujoco)
LIFT = 0.012         # foot clearance (m)
LEAN = 0.20          # pelvis roll toward the loaded foot at full load (rad)
# hobby-servo stiffness, N.m/rad: a position servo reaches its stall torque a fixed angle error off target, so
# its stiffness scales with its stall torque. MG92B 6 N.m/rad at 0.343 N.m (P0 measures it); the MuJoCo actuators
# use the same per-joint values
KP_PER_NM = 6.0 / 0.343
SERVO_KP = {j: KP_PER_NM * _J[j]["effort"] for j in JOINTS}
SAG_FF = 1.25        # command += SAG_FF * hold torque / SERVO_KP; above 1 so an underestimated load errs stiff
H0_LEANED = L1 + L2 - 2 * HIP_Y * math.sin(LEAN) - 0.002   # at full lean: the free leg reaches the floor, 2 mm spare
T_SHIFT, T_SWING, T_SETTLE = 1.2, 1.2, 0.3   # seconds per step phase
T_TURN = 1.2         # turning swings are this much slower
WALK_STEPS = 8
TURN = math.radians(15)   # yaw per pair of turning steps
TURN_PAIRS = 4            # 4 x 15 deg = 60 deg to the left
T_CROUCH = 1.0
T_WAVE = 5.0
ELBOW = -0.4


def smooth(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


def blend(a, b, w):
    keys = set(a) | set(b)
    return {k: (1 - w) * a.get(k, 0.0) + w * b.get(k, 0.0) for k in keys}


def leg_ik(x, z):
    """Sagittal IK. Ankle pitch axis at x forward, z below the hip pitch axis. Returns hip_pitch, knee,
    ankle_pitch that keep the foot parallel to the pelvis."""
    d = min(math.hypot(x, z), L1 + L2 - 1e-6)
    alpha = math.atan2(x, z)
    beta = math.acos((d * d + L1 * L1 - L2 * L2) / (2 * L1 * d))
    gamma = math.acos((L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2))
    return -(alpha + beta), math.pi - gamma, alpha + beta - (math.pi - gamma)


def leg_q(side, v, dyaw, droll):
    """Joint angles of one leg: sole at v (pelvis frame), foot yawed by dyaw and rolled by droll from the pelvis."""
    w = Rz(-dyaw) @ (v - PIVOT[side]) - Rx(droll) @ np.array([0.0, 0.0, -SOLE])     # ankle roll axis point
    Z = math.hypot(w[1], w[2])
    r = math.atan2(w[1], -w[2])
    hp, kn, ap = leg_ik(w[0] - ANKLE_X, Z - ROLL_UP - ROLL_DOWN)
    return {f"{side}_hip_yaw": dyaw, f"{side}_hip_roll": r, f"{side}_hip_pitch": hp, f"{side}_knee": kn,
            f"{side}_ankle_pitch": ap, f"{side}_ankle_roll": droll - r}


def whole_body(com_xy, z, yaw, roll, feet, upper, guess=(0.0, 0.0)):
    """Joint angles that put the whole-body CoM over com_xy (world), pelvis at height z with yaw and roll, soles at
    feet = {side: (x, y, z, yaw)}. Returns (pelvis xy, q, fk, pelvis rotation)."""
    Rb = Rz(yaw) @ Rx(roll)
    bx, by = guess
    for _ in range(20):
        q = dict(upper)
        for s, (fx, fy, fz, fyaw) in feet.items():
            q.update(leg_q(s, Rb.T @ np.array([fx - bx, fy - by, fz - z]), fyaw - yaw, -roll))
        T = MODEL.fk(q)
        c = Rb @ MODEL.com_of(T)
        ex, ey = com_xy[0] - bx - c[0], com_xy[1] - by - c[1]
        bx, by = bx + ex, by + ey
        if abs(ex) + abs(ey) < 1e-6:
            break
    return (bx, by), q, T, Rb


class Plan:
    """A sequence of segments, each a function u in [0, 1] -> (CoM xy, pelvis yaw, feet, upper body)."""

    def __init__(self, walk_steps=WALK_STEPS, stride=STEP, scale=1.0, turn_pairs=TURN_PAIRS, wave=True):
        self.feet = {"l": [0.0, HIP_Y, 0.0, 0.0], "r": [0.0, -HIP_Y, 0.0, 0.0]}
        self.com, self.yaw, self.segs, self.marks = np.zeros(2), 0.0, [], {}
        up = {"l_elbow": ELBOW, "r_elbow": ELBOW}
        ts, tw, tl = T_SHIFT * scale, T_SWING * scale, T_SETTLE * scale
        self.marks["walk"] = 0.0
        for k in range(walk_steps):
            sw, st = ("r", "l") if k % 2 == 0 else ("l", "r")
            self.shift(st, 0.0, ts)
            x = self.feet[st][0] + (0.0 if k == walk_steps - 1 else stride / 2 if k == 0 else stride)
            y = self.feet[st][1] + (2 if sw == "l" else -2) * HIP_Y

            def arms(u, k=k):
                a = 0.25 * math.sin(math.pi * (k + u))
                return dict(up, l_shoulder_pitch=a, r_shoulder_pitch=-a)
            self.swing(sw, (x, y, 0.0, 0.0), tw, tl, arms)
        self.shift(None, 0.0, ts)
        self.marks["turn"] = self.duration
        centre, yaw = (np.array(self.feet["l"][:2]) + self.feet["r"][:2]) / 2, 0.0
        for _ in range(turn_pairs):
            for sw, st in (("l", "r"), ("r", "l")):
                self.shift(st, self.feet[st][3], ts)            # pelvis faces the stance foot: no twist on it
                yaw_sw = yaw + TURN
                p = centre + Rz(yaw_sw)[:2, :2] @ np.array([0.0, HIP_Y if sw == "l" else -HIP_Y])
                self.swing(sw, (p[0], p[1], 0.0, yaw_sw), tw * T_TURN, tl)
            yaw += TURN
        if turn_pairs:
            self.shift(None, yaw, ts)
        self.marks["wave"] = self.duration
        if wave:
            self.shift(None, yaw, T_WAVE, self.wave)
        self.marks["end"] = self.duration

    @property
    def duration(self):
        return sum(s[0] for s in self.segs)

    def snap(self):
        return {k: tuple(v) for k, v in self.feet.items()}

    def shift(self, stance, yaw_to, T, upper=None):
        """Move the CoM over the stance foot (None: midway between the feet) and turn the pelvis to yaw_to."""
        ft, c0, y0 = self.snap(), self.com.copy(), self.yaw
        target = np.array(ft[stance][:2]) if stance else (np.array(ft["l"][:2]) + ft["r"][:2]) / 2
        upper = upper or (lambda u: {"l_elbow": ELBOW, "r_elbow": ELBOW})
        self.segs.append((T, lambda u: (c0 + (target - c0) * smooth(u), y0 + (yaw_to - y0) * smooth(u), ft,
                                        upper(u))))
        self.com, self.yaw = target, yaw_to

    def swing(self, side, to, T, settle, upper=None):
        """Swing one foot to `to` = (x, y, z, yaw) with the CoM held; then both feet down for `settle` s."""
        ft, com, yaw, fr = self.snap(), self.com.copy(), self.yaw, self.feet[side]
        upper = upper or (lambda u: {"l_elbow": ELBOW, "r_elbow": ELBOW})

        def seg(u):
            v, f = smooth(u), dict(ft)
            f[side] = (fr[0] + (to[0] - fr[0]) * v, fr[1] + (to[1] - fr[1]) * v, LIFT * math.sin(math.pi * u),
                       fr[3] + (to[3] - fr[3]) * v)
            return com, yaw, f, upper(u)
        self.segs.append((T, seg))
        self.feet[side] = list(to)
        ft2 = self.snap()
        self.segs.append((settle, lambda u: (com, yaw, ft2, upper(1.0))))

    @staticmethod
    def wave(u):
        t = u * T_WAVE
        w = smooth(t / 1.5)
        return {"r_shoulder_pitch": -2.6 * w, "r_shoulder_roll": -0.3 * w,
                "r_elbow": ELBOW + w * (-0.1 + 0.3 * math.sin(2 * math.pi * t)), "l_elbow": ELBOW,
                "neck_yaw": 0.4 * w * math.sin(2 * math.pi * 0.25 * t)}

    def sample(self, t):
        """(CoM xy, pelvis yaw, feet, upper) at t seconds into the plan."""
        for T, fn in self.segs:
            if t <= T:
                return fn(t / T if T > 0 else 1.0)
            t -= T
        return self.segs[-1][1](1.0)


def pose(com, yaw, feet, upper, z=None, guess=(0.0, 0.0), sag=True):
    """Joint angles for one planned instant: the servo command (sag included), or the planned pose itself
    (sag=False, what the robot should look like). Returns (q, pelvis xy)."""
    a, b = np.array(feet["l"][:2]), np.array(feet["r"][:2])
    d = a - b
    f = float(np.clip(2 * np.dot(np.asarray(com) - (a + b) / 2, d) / np.dot(d, d), -1.0, 1.0))  # +1: all on left
    if z is None:                       # the leaned pelvis lifts the free hip: crouch so the free foot still lands
        z = SOLE + ROLL_DOWN + H0 - (H0 - H0_LEANED) * abs(f)
    xy, q, T, Rb = whole_body(com, z, yaw, -LEAN * f, feet, upper, guess)
    down = [s for s, ft in feet.items() if ft[2] < 1e-6]
    load = {down[0]: 1.0} if len(down) == 1 else {"l": (1 + f) / 2, "r": (1 - f) / 2}
    tau = MODEL.hold_torques(T, Rb, load) if sag else {}
    cmd = {j: q.get(j, 0.0) + SAG_FF * tau.get(j, 0.0) / SERVO_KP[j] for j in JOINTS}
    return {j: min(max(v, LIMIT[j][0]), LIMIT[j][1]) for j, v in cmd.items()}, xy


def crouch_pose(w, guess=(0.0, 0.0), sag=True):
    """From straight legs (w = 0) to the gait's start pose (w = 1), CoM between the feet."""
    feet = {"l": (0.0, HIP_Y, 0.0, 0.0), "r": (0.0, -HIP_Y, 0.0, 0.0)}
    z0 = SOLE + ROLL_DOWN + L1 + L2 - 1e-4
    return pose(np.zeros(2), 0.0, feet, {"l_elbow": ELBOW * w, "r_elbow": ELBOW * w},
                z0 + (SOLE + ROLL_DOWN + H0 - z0) * smooth(w), guess, sag)


class Demo:
    def __init__(self, plan=None):
        self.plan = plan or Plan()
        self.guess = (0.0, 0.0)

    def targets(self, t, sag=True):
        if t < T_CROUCH:
            q, self.guess = crouch_pose(t / T_CROUCH, self.guess, sag)
        else:
            q, self.guess = pose(*self.plan.sample(t - T_CROUCH), guess=self.guess, sag=sag)
        return q


_DEMO = Demo()
T_WALK0 = T_CROUCH
T_TURN0 = T_CROUCH + _DEMO.plan.marks["turn"]
T_WAVE0 = T_CROUCH + _DEMO.plan.marks["wave"]
T_END = T_CROUCH + _DEMO.plan.marks["end"]
T_STEP = T_SHIFT + T_SWING + T_SETTLE


def targets(t, sag=True):
    """The demo: joint commands (dict) t seconds after it starts; sag=False gives the planned pose."""
    return _DEMO.targets(t, sag)


def walk(t):
    return targets(T_WALK0 + t)


def turn(t):
    return targets(T_TURN0 + t)


def single_support(stance="l"):
    """The planned pose halfway through a swing, standing on `stance` (static load checks)."""
    plan = Plan(walk_steps=1 if stance == "l" else 2, turn_pairs=0, wave=False)
    T, fn = plan.segs[1 if stance == "l" else 4]                   # each step: shift, swing, settle
    return pose(*fn(0.5), sag=False)[0]


_CYCLES = {}


def walk_continuous(t, step_len=STEP, t_step=T_STEP, **_):
    """Endless walking from the start pose: two steps to get going, then periodic in t (two steps of `t_step`
    seconds and `step_len` stride per cycle)."""
    key = (round(step_len, 4), round(t_step, 3))
    if key not in _CYCLES:
        plan = Plan(walk_steps=6, stride=step_len, scale=t_step / T_STEP, turn_pairs=0, wave=False)
        _CYCLES[key] = (Demo(plan), sum(s[0] for s in plan.segs[:6]), 2 * t_step)   # steps 3 and 4: steady
    demo, start, period = _CYCLES[key]
    return demo.targets(T_CROUCH + (t if t < start else start + (t - start) % period))


def trajectory_points(sample_rate=50.0, start_delay=2.0):
    """The whole demo as trajectory samples: times (s from start) and positions in JOINTS order. Before
    start_delay every joint is 0 (straight legs, as spawned)."""
    dt = 1.0 / sample_rate
    demo = Demo(_DEMO.plan)
    times, positions = [], []
    for i in range(int(round((start_delay + T_END) / dt)) + 1):
        t = i * dt
        q = demo.targets(t - start_delay) if t > start_delay else {}
        times.append(t)
        positions.append([float(q.get(j, 0.0)) for j in JOINTS])
    return times, positions


def make_node():
    """ROS imports live here so the gait math above can be imported without ROS (MuJoCo, firmware tables, docs)."""
    from builtin_interfaces.msg import Duration
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    class BazanGait(Node):
        def __init__(self):
            super().__init__("bazan_gait")
            self.declare_parameter("sample_rate", 50.0)
            self.declare_parameter("start_delay", 2.0)
            self.declare_parameter("loop", False)
            self.pub = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
            self.next_send = None
            self.create_timer(0.1, self.tick)

        def trajectory(self):
            msg = JointTrajectory(joint_names=JOINTS)        # zero stamp: start on arrival
            for t, q in zip(*trajectory_points(self.get_parameter("sample_rate").value,
                                               self.get_parameter("start_delay").value)):
                ns = int(round(t * 1e9))
                stamp = Duration(sec=ns // 10**9, nanosec=ns % 10**9)
                msg.points.append(JointTrajectoryPoint(positions=q, time_from_start=stamp))
            return msg

        def tick(self):
            now = self.get_clock().now().nanoseconds * 1e-9
            if now <= 0.0 or self.pub.get_subscription_count() == 0:
                return                                   # wait for /clock and the controller
            if self.next_send is not None and (now < self.next_send or not self.get_parameter("loop").value):
                return
            msg = self.trajectory()
            self.pub.publish(msg)
            length = self.get_parameter("start_delay").value + T_END
            self.next_send = now + length
            self.get_logger().info(f"sent {len(msg.points)} points, {length:.1f} s "
                                   f"(walk, turn, wave start after {self.get_parameter('start_delay').value:.1f} s)")

    return BazanGait()


def main():
    import rclpy
    import rclpy.executors
    rclpy.init()
    node = make_node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
