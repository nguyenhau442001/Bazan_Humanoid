#!/usr/bin/env python3
"""ROS 2 locomotion node for Bazan: runs a trained ONNX policy at 50 Hz exactly as it was trained.

The observation, the action -> joint target mapping (EMA filter, residual on the reference gait, scales) and the
INA226 / NTC models all come from bazan_policy_io.py, the module the training environment uses. The mapping
settings are read from the ONNX file (written by train_bazan_ppo.py); policies without robot=bazan_cad (trained
on the retired twin) are refused.

Subscribes:
    /joint_states   sensor_msgs/JointState (position, velocity, effort = servo torque)
    /imu            sensor_msgs/Imu (orientation, gyro, specific force) in imu_link
    /foot_forces    std_msgs/Float32MultiArray, N under L heel, L toe, R heel, R toe
    /tof            sensor_msgs/Range (VL53L0X)
    /cmd_vel        geometry_msgs/Twist
Publishes:
    /position_controller/commands                   std_msgs/Float64MultiArray, 20 targets (bazan_gait.JOINTS)
    /joint_trajectory_controller/joint_trajectory   the same as a one-point trajectory, when someone listens

    ros2 run humanoid_control bazan_rl_node.py --ros-args -p model_path:=/path/to/policy.onnx
"""
import os
from pathlib import Path
import sys

import numpy as np

try:
    import onnxruntime as ort
except ImportError:
    ort = None

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
try:
    import bazan_gait as gait
except ImportError:
    import mico_gait as gait  # noqa: E402
import bazan_policy_io as io  # noqa: E402

LEG_JOINTS, ARM_6_JOINTS, NECK_JOINTS, OBS_ARM_JOINTS = io.LEG_JOINTS, io.ARM_JOINTS, io.NECK_JOINTS, \
    io.OBS_ARM_JOINTS


def find_default_model():
    """Locate bazan_policy.onnx across install directory or source repository."""
    # 1. Try ament package share
    try:
        from ament_index_python.packages import get_package_share_directory
        share_model = Path(get_package_share_directory("humanoid_control")) / "models" / "bazan_policy.onnx"
        if share_model.exists():
            return str(share_model)
    except Exception:
        pass

    # 2. Search upward from this file
    curr = HERE
    for _ in range(6):
        cand1 = curr / "src" / "humanoid_control" / "models" / "bazan_policy.onnx"
        if cand1.exists():
            return str(cand1)
        cand2 = curr / "models" / "bazan_policy.onnx"
        if cand2.exists():
            return str(cand2)
        if curr.parent == curr:
            break
        curr = curr.parent

    # 3. Fallback to current working directory
    cand3 = Path("src/humanoid_control/models/bazan_policy.onnx")
    if cand3.exists():
        return str(cand3.resolve())

    return os.path.expanduser("~/GitHub/Synapse_Humanoid/src/humanoid_control/models/bazan_policy.onnx")


def quat_to_rot_matrix(qx, qy, qz, qw):
    """Convert quaternion [x, y, z, w] to 3x3 rotation matrix."""
    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)]
    ])


def read_policy_meta(session):
    """Training settings stored in the ONNX file, as typed values. Raises on a policy of another robot."""
    raw = dict(session.get_modelmeta().custom_metadata_map)
    if raw.get("robot") != io.ROBOT_TAG:
        raise RuntimeError(f"policy was not trained on {io.ROBOT_TAG} (metadata robot={raw.get('robot')}); retrain "
                           "it with train_bazan_ppo.py")
    flag = lambda k: raw.get(k, "False") == "True"                                     # noqa: E731
    return dict(residual=flag("residual"), straight_line=flag("straight_line"), perception=flag("perception"),
                action_scale=float(raw["action_scale"]), action_filter=float(raw["action_filter"]),
                control_dt=float(raw["control_dt"]))


class PolicyRunner:
    """Everything between sensor values and joint targets, without ROS (tested offline)."""

    def __init__(self, session):
        self.session = session
        self.meta = read_policy_meta(session)
        self.input_name = session.get_inputs()[0].name
        self.obs_dim = int(session.get_inputs()[0].shape[-1])
        self.act_dim = int(session.get_outputs()[0].shape[-1])
        self.map = io.ActionMap(self.meta["action_scale"], self.meta["action_filter"], self.meta["residual"],
                                self.meta["straight_line"])
        self.power, self.thermal, self.camera = io.PowerModel(), io.ThermalModel(), io.CameraProxy()
        self.hip = [gait.JOINTS.index(j) for j in ("l_hip_roll", "r_hip_roll")]
        self.leg = [gait.JOINTS.index(j) for j in LEG_JOINTS]
        self.arm = [gait.JOINTS.index(j) for j in OBS_ARM_JOINTS]
        self.leg_q_nominal = self.map.q_nominal[self.leg]
        self.reset()

    def reset(self):
        self.t, self.prev_action = 0.0, np.zeros(self.act_dim)
        self.map.reset()
        self.camera.reset()

    def observe(self, s):
        """s: q, dq, effort (20, JOINTS order), gravity, gyro, acc_z, foot_forces (4), tof_m, command (3)."""
        if self.t > 0.0:            # temperatures integrate the load since the previous period (as in training)
            self.thermal.step(self.meta["control_dt"], s["effort"][self.hip], io.STALL_NM[self.hip])
        leg_q_err = s["q"][self.leg] - self.leg_q_nominal
        leg_dq = s["dq"][self.leg]
        if self.obs_dim == 64:
            bus_v, current = self.power.read(s["effort"])
            return io.obs64(s["gravity"], s["gyro"], s["acc_z"], leg_q_err, leg_dq, s["q"][self.arm],
                            self.prev_action[:12], s["command"], self.t, s["foot_forces"], s["tof_m"],
                            self.camera(s["tof_m"]), bus_v, current, self.thermal.temps)
        obs = [s["gravity"], s["gyro"], leg_q_err, leg_dq, self.prev_action[:12], s["command"]]
        if self.obs_dim in (47, 51):
            obs.append(io.phase_vec(self.t))
        if self.obs_dim in (49, 51):
            obs.append(io.fsr_vec(s["foot_forces"]))
        return np.concatenate(obs).astype(np.float32)

    def step(self, s):
        """One control period: observation -> policy -> 20 joint targets."""
        obs = self.observe(s)
        action = np.clip(self.session.run(None, {self.input_name: obs[None].astype(np.float32)})[0][0], -1, 1)
        q = self.map(action, self.t, s["command"])
        self.prev_action = action.copy()
        self.t += self.meta["control_dt"]
        return q


def make_rl_node():
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, JointState, Range
    from std_msgs.msg import Float32MultiArray, Float64MultiArray
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    class BazanRLNode(Node):
        def __init__(self):
            super().__init__("bazan_rl_node")
            model_path = self.declare_parameter("model_path", find_default_model()).value
            self.get_logger().info(f"Loading ONNX policy from: {model_path}")
            if not Path(model_path).exists():
                raise FileNotFoundError(f"policy not found: {model_path} (no policy is trained for the CAD robot "
                                        "yet: run train_bazan_ppo.py)")
            if ort is None:
                raise RuntimeError("onnxruntime is required to run bazan_rl_node")
            self.runner = PolicyRunner(ort.InferenceSession(model_path))
            m = self.runner.meta
            self.get_logger().info(f"policy: residual={m['residual']} action_scale={m['action_scale']} "
                                   f"filter={m['action_filter']} obs={self.runner.obs_dim} act={self.runner.act_dim}")
            n = len(gait.JOINTS)
            self.s = dict(q=np.zeros(n), dq=np.zeros(n), effort=np.zeros(n), gravity=np.array([0.0, 0.0, -1.0]),
                          gyro=np.zeros(3), acc_z=9.81, foot_forces=np.zeros(4), tof_m=io.TOF_MAX,
                          command=np.zeros(3))
            self.has_joint_state = self.has_imu = self.walking = False

            self.pub_pos = self.create_publisher(Float64MultiArray, "/position_controller/commands", 10)
            self.pub_traj = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
            self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
            self.create_subscription(Imu, "/imu", self.on_imu, 10)
            self.create_subscription(Float32MultiArray, "/foot_forces", self.on_feet, 10)
            self.create_subscription(Range, "/tof", self.on_tof, 10)
            self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
            self.msgs = (Float64MultiArray, JointTrajectory, JointTrajectoryPoint, Duration)
            self.create_timer(m["control_dt"], self.control_loop)
            self.get_logger().info(f"BazanRLNode ready @ {1.0 / m['control_dt']:.0f} Hz")

        def on_joints(self, msg):
            for name, *vals in zip(msg.name, msg.position, msg.velocity, msg.effort or [0.0] * len(msg.name)):
                if name in gait.JOINTS:
                    i = gait.JOINTS.index(name)
                    self.s["q"][i], self.s["dq"][i], self.s["effort"][i] = vals
            self.has_joint_state = True

        def on_imu(self, msg):
            q = msg.orientation
            self.s["gravity"] = quat_to_rot_matrix(q.x, q.y, q.z, q.w).T @ np.array([0.0, 0.0, -1.0])
            g = msg.angular_velocity
            self.s["gyro"] = np.array([g.x, g.y, g.z])
            self.s["acc_z"] = msg.linear_acceleration.z
            self.has_imu = True

        def on_feet(self, msg):
            self.s["foot_forces"] = np.asarray(msg.data, float)

        def on_tof(self, msg):
            self.s["tof_m"] = min(float(msg.range), io.TOF_MAX)

        def on_cmd(self, msg):
            self.s["command"] = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

        def control_loop(self):
            if not (self.has_joint_state and self.has_imu):
                return
            Float64MultiArray, JointTrajectory, JointTrajectoryPoint, Duration = self.msgs
            c = self.s["command"]
            moving = abs(c[0]) > 0.2 * io.V_REF or abs(c[1]) > 0.2 * io.V_REF or abs(c[2]) > 0.2 * io.W_REF
            if not moving:                                      # hold the gait's start pose, restart the clock
                if self.walking:
                    self.runner.reset()
                    self.walking = False
                q = self.runner.map.q_nominal
            else:
                self.walking = True
                q = self.runner.step(self.s)
            self.pub_pos.publish(Float64MultiArray(data=[float(v) for v in q]))
            if self.pub_traj.get_subscription_count() > 0:
                ns = int(round(self.runner.meta["control_dt"] * 1e9))
                pt = JointTrajectoryPoint(positions=[float(v) for v in q],
                                          time_from_start=Duration(sec=ns // 10**9, nanosec=ns % 10**9))
                self.pub_traj.publish(JointTrajectory(joint_names=gait.JOINTS, points=[pt]))

    return BazanRLNode()


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = make_rl_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


# Backward-compatibility alias
make_mico_rl_node = make_rl_node
