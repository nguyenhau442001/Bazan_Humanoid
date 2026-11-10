#!/usr/bin/env python3
"""Mico quasi-static gait for MuJoCo / the real robot: crouch -> walk 8 steps -> turn left 60 deg -> wave.

Sends the whole demo as one trajectory_msgs/JointTrajectory (50 Hz samples) to
/joint_trajectory_controller/joint_trajectory. The MuJoCo bridge interpolates it on sim time,
so every run in MuJoCo is identical. Joint order: JOINTS below.

    ros2 run humanoid_control mico_gait.py
    ros2 run humanoid_control mico_gait.py --ros-args -p loop:=true

Every step is the same 4-phase cycle: shift the CoM over the stance foot (hip roll + ankle roll), lift and
move the swing foot, put it down, shift back. Turning reuses the cycle with the hip-yaw joints.
"""
import math

JOINTS = ["l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle_pitch", "l_ankle_roll",
          "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle_pitch", "r_ankle_roll",
          "l_shoulder_pitch", "l_shoulder_roll", "l_elbow", "r_shoulder_pitch", "r_shoulder_roll", "r_elbow",
          "neck_yaw", "head_pitch"]

# ---------------------------------------------------------------- gait parameters
L1 = L2 = 0.045      # thigh, shin (m): hip point H -> knee -> ankle point A
H0 = 0.087           # nominal H-to-A height (slight crouch)
STEP = 0.024         # step length (m)
LIFT = 0.010         # foot clearance (m)
SWAY = 0.20          # hip/ankle roll during single support (rad), shifts the CoM over the stance foot
LEAN = -0.10         # extra lean toward the stance leg (rad); unloads the stance hip roll
T_STEP = 1.6         # seconds per step
WALK_STEPS = 8
TURN = math.radians(15)   # body rotation per pair of turning steps
TURN_PAIRS = 4            # 4 x 15 deg = 60 deg to the left
T_TURN_STEP = 2.0         # turning steps are slower: the planted foot pivots under load


def leg_ik(x, z):
    """Sagittal IK. Ankle point A at x forward, z below hip point H. Returns hip_pitch, knee, ankle_pitch
    that keep the foot flat."""
    d = min(math.hypot(x, z), L1 + L2 - 1e-6)
    alpha = math.atan2(x, z)
    beta = math.acos(d / (2 * L1))
    return -(alpha + beta), 2 * beta, alpha - beta


def smooth(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


def step_pose(u, swing, x0, x1, yaw_sw, yaw_st, sway=SWAY, lean=LEAN):
    """One step at phase u in [0, 1]. swing: 'l' or 'r'; x0 -> x1: swing-foot x relative to its hip;
    yaw_sw, yaw_st: (start, end) hip-yaw angles of the swing and stance leg."""
    stance = "l" if swing == "r" else "r"
    q = {}
    s = smooth(u / 0.25) if u < 0.25 else (1.0 if u < 0.75 else 1 - smooth((u - 0.75) / 0.25))
    sign = 1 if stance == "l" else -1
    r = sway * s * sign
    lean_val = lean * s * sign
    v = smooth((u - 0.25) / 0.5)
    xs = x0 + (x1 - x0) * v
    zs = H0 - LIFT * math.sin(math.pi * v)
    xt = -x0 + (x0 - x1) * v
    for side, (x, z), (y0, y1) in ((swing, (xs, zs), yaw_sw), (stance, (xt, H0), yaw_st)):
        q[f"{side}_hip_pitch"], q[f"{side}_knee"], q[f"{side}_ankle_pitch"] = leg_ik(x, z)
        q[f"{side}_ankle_roll"], q[f"{side}_hip_roll"] = r, -r - lean_val
        q[f"{side}_hip_yaw"] = y0 + (y1 - y0) * v
    return q


def walk(t):
    k = min(int(t // T_STEP), WALK_STEPS - 1)
    u = 1.0 if t >= WALK_STEPS * T_STEP else (t % T_STEP) / T_STEP
    swing = "r" if k % 2 == 0 else "l"
    x0 = 0.0 if k == 0 else -STEP / 2
    x1 = 0.0 if k == WALK_STEPS - 1 else STEP / 2
    q = step_pose(u, swing, x0, x1, (0, 0), (0, 0))
    q["l_shoulder_pitch"] = 0.25 * math.sin(math.pi * (k + u))
    q["r_shoulder_pitch"] = -q["l_shoulder_pitch"]
    q["l_elbow"] = q["r_elbow"] = -0.4
    return q


def walk_continuous(t, step_len=STEP, t_step=T_STEP, sway=SWAY, lean=LEAN):
    """Continuous, infinite periodic bipedal walking gait without step limits."""
    t_cycle = 2 * t_step
    t_mod = t % t_cycle
    k = int(t_mod // t_step)
    u = (t_mod % t_step) / t_step
    swing = "r" if k % 2 == 0 else "l"
    x0 = -step_len / 2
    x1 = step_len / 2
    q = step_pose(u, swing, x0, x1, (0, 0), (0, 0), sway=sway, lean=lean)
    q["l_shoulder_pitch"] = 0.25 * math.sin(math.pi * (k + u))
    q["r_shoulder_pitch"] = -q["l_shoulder_pitch"]
    q["l_elbow"] = q["r_elbow"] = -0.4
    return q


def turn(t):
    """Turn left in place: lift the left foot and open its hip yaw, then lift the right foot while the
    left hip yaw returns to 0; the planted left foot stays put, so the body rotates +TURN per pair."""
    n = 2 * TURN_PAIRS
    k = min(int(t // T_TURN_STEP), n - 1)
    u = 1.0 if t >= n * T_TURN_STEP else (t % T_TURN_STEP) / T_TURN_STEP
    if k % 2 == 0:
        q = step_pose(u, "l", 0, 0, (0, TURN), (0, 0))
    else:
        q = step_pose(u, "r", 0, 0, (0, 0), (TURN, 0))
    q["l_elbow"] = q["r_elbow"] = -0.4
    return q


def wave(t):
    q = {f"{s}_{j}": a for s in "lr" for j, a in zip(("hip_pitch", "knee", "ankle_pitch"), leg_ik(0, H0))}
    q["r_shoulder_pitch"] = -2.6
    q["r_shoulder_roll"] = -0.3
    q["r_elbow"] = -0.5 + 0.3 * math.sin(2 * math.pi * 1.0 * t)
    q["l_elbow"] = -0.4
    q["neck_yaw"] = 0.4 * math.sin(2 * math.pi * 0.25 * t)
    return q


def crouch(t):
    w = smooth(t / 1.0)
    hp, kn, ap = leg_ik(0, 0.090 + (H0 - 0.090) * w)
    q = {f"{s}_{j}": a for s in "lr" for j, a in zip(("hip_pitch", "knee", "ankle_pitch"), (hp, kn, ap))}
    q["l_elbow"] = q["r_elbow"] = -0.4 * w
    return q


def blend(a, b, w):
    keys = set(a) | set(b)
    return {k: (1 - w) * a.get(k, 0.0) + w * b.get(k, 0.0) for k in keys}


T_WALK0 = 1.0
T_TURN0 = T_WALK0 + WALK_STEPS * T_STEP + 0.5
T_WAVE0 = T_TURN0 + 2 * TURN_PAIRS * T_TURN_STEP + 0.5
T_END = T_WAVE0 + 5.0


def targets(t):
    if t < T_WALK0:
        return crouch(t)
    if t < T_TURN0:
        return walk(t - T_WALK0)
    if t < T_WAVE0:
        return turn(t - T_TURN0)
    return blend(turn(1e9), wave(t - T_WAVE0), smooth((t - T_WAVE0) / 1.5))


def trajectory_points(sample_rate=50.0, start_delay=2.0):
    """The whole demo as trajectory samples: times (s from start) and positions in JOINTS order. Before
    start_delay every joint is 0 (straight legs, as spawned)."""
    dt = 1.0 / sample_rate
    n = int(round((start_delay + T_END) / dt))
    times, positions = [], []
    for i in range(n + 1):
        t = i * dt
        q = targets(max(t - start_delay, 0.0)) if t > start_delay else {}
        times.append(t)
        positions.append([float(q.get(j, 0.0)) for j in JOINTS])
    return times, positions


def make_node():
    """ROS imports live here so the gait math above can be imported without ROS (sim/mini_humanoid/kin_dyn.py)."""
    from builtin_interfaces.msg import Duration
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    class MicoGait(Node):
        def __init__(self):
            super().__init__("mico_gait")
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

    return MicoGait()


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
