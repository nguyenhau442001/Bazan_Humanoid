#!/usr/bin/env python3
"""ROS 2 Real-Time Locomotion Controller Node for Mico Humanoid.

Executes a trained lightweight ONNX policy at 50 Hz.
Subscribes to:
- /joint_states (sensor_msgs/msg/JointState)
- /imu (sensor_msgs/msg/Imu)
- /cmd_vel (geometry_msgs/msg/Twist)

Publishes to:
- /position_controller/commands (std_msgs/msg/Float64MultiArray)
- /joint_trajectory_controller/joint_trajectory (trajectory_msgs/msg/JointTrajectory)

Usage:
    ros2 run humanoid_control mico_rl_node.py
"""
import math
import os
from pathlib import Path
import sys

import numpy as np

try:
    import onnxruntime as ort
except ImportError:
    ort = None

HERE = Path(__file__).resolve().parent
REPO = HERE
while REPO.parent != REPO and not (REPO / ".git").exists() and not (REPO / "AGENTS.md").exists():
    REPO = REPO.parent
sys.path.insert(0, str(HERE))
import mico_gait as gait  # noqa: E402

LEG_JOINTS = [
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle_pitch", "l_ankle_roll",
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle_pitch", "r_ankle_roll"
]

ARM_JOINTS = [
    "l_shoulder_pitch", "l_elbow", "r_shoulder_pitch", "r_elbow"
]


def find_default_model():
    """Locate mico_policy.onnx across install directory or source repository."""
    # 1. Try ament package share
    try:
        from ament_index_python.packages import get_package_share_directory
        share_model = Path(get_package_share_directory("humanoid_control")) / "models" / "mico_policy.onnx"
        if share_model.exists():
            return str(share_model)
    except Exception:
        pass

    # 2. Search upward from this file
    curr = HERE
    for _ in range(6):
        cand1 = curr / "src" / "humanoid_control" / "models" / "mico_policy.onnx"
        if cand1.exists():
            return str(cand1)
        cand2 = curr / "models" / "mico_policy.onnx"
        if cand2.exists():
            return str(cand2)
        if curr.parent == curr:
            break
        curr = curr.parent

    # 3. Fallback to current working directory
    cand3 = Path("src/humanoid_control/models/mico_policy.onnx")
    if cand3.exists():
        return str(cand3.resolve())

    return os.path.expanduser("~/GitHub/Synapse_Humanoid/src/humanoid_control/models/mico_policy.onnx")


def quat_to_rot_matrix(qx, qy, qz, qw):
    """Convert quaternion [x, y, z, w] to 3x3 rotation matrix."""
    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)]
    ])


def make_rl_node():
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import Float64MultiArray
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    class MicoRLNode(Node):
        def __init__(self):
            super().__init__("mico_rl_node", automatically_declare_parameters_from_overrides=True)

            def declare_if_not(name, default):
                if not self.has_parameter(name):
                    return self.declare_parameter(name, default)
                return self.get_parameter(name)

            default_model = find_default_model()
            declare_if_not("model_path", default_model)
            declare_if_not("action_scale", 0.25)
            declare_if_not("control_rate", 50.0)
            declare_if_not("default_vx", 0.0)
            declare_if_not("use_sim_time", True)

            model_path = self.get_parameter("model_path").value
            self.get_logger().info(f"Loading ONNX policy from: {model_path}")
            if not Path(model_path).exists():
                self.get_logger().error(f"Policy file not found: {model_path}")
                raise FileNotFoundError(model_path)

            if ort is None:
                raise RuntimeError("onnxruntime is required to run MicoRLNode")
            self.session = ort.InferenceSession(model_path)
            self.input_name = self.session.get_inputs()[0].name
            self.expected_obs_dim = int(self.session.get_inputs()[0].shape[1])
            self.expected_act_dim = int(self.session.get_outputs()[0].shape[1])
            self.action_scale = float(self.get_parameter("action_scale").value)
            rate_hz = float(self.get_parameter("control_rate").value)
            self.dt = 1.0 / rate_hz

            # Joint indices & nominal pose
            self.joint_positions = {j: 0.0 for j in gait.JOINTS}
            self.joint_velocities = {j: 0.0 for j in gait.JOINTS}

            self.q_nominal = np.zeros(len(gait.JOINTS), dtype=np.float32)
            hp, kn, ap = gait.leg_ik(0, gait.H0)
            for side in ("l", "r"):
                self.q_nominal[gait.JOINTS.index(f"{side}_hip_pitch")] = hp
                self.q_nominal[gait.JOINTS.index(f"{side}_knee")] = kn
                self.q_nominal[gait.JOINTS.index(f"{side}_ankle_pitch")] = ap
            self.q_nominal[gait.JOINTS.index("l_elbow")] = -0.4
            self.q_nominal[gait.JOINTS.index("r_elbow")] = -0.4

            self.leg_q_nominal = np.array([self.q_nominal[gait.JOINTS.index(j)] for j in LEG_JOINTS])
            self.arm_q_nominal = np.array([self.q_nominal[gait.JOINTS.index(j)] for j in ARM_JOINTS])

            # State cache
            self.prev_action = np.zeros(self.expected_act_dim, dtype=np.float32)
            self.command = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            self.proj_gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)
            self.omega_base = np.zeros(3, dtype=np.float32)
            self.step_count = 0

            self.has_joint_state = False
            self.has_imu = False
            self.logged_active = False

            # ROS 2 Pub / Sub
            self.pub_pos = self.create_publisher(
                Float64MultiArray, "/position_controller/commands", 10
            )
            self.pub_traj = self.create_publisher(
                JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
            )
            self.sub_joint = self.create_subscription(
                JointState, "/joint_states", self.joint_callback, 10
            )
            self.sub_imu = self.create_subscription(
                Imu, "/imu", self.imu_callback, 10
            )
            self.sub_cmd = self.create_subscription(
                Twist, "/cmd_vel", self.cmd_vel_callback, 10
            )

            self.timer = self.create_timer(self.dt, self.control_loop)
            self.get_logger().info(f"MicoRLNode ready @ {rate_hz:.1f} Hz")

        def joint_callback(self, msg: JointState):
            for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
                if name in self.joint_positions:
                    self.joint_positions[name] = pos
                    self.joint_velocities[name] = vel
            self.has_joint_state = True

        def imu_callback(self, msg: Imu):
            q = msg.orientation
            norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            if norm > 1e-6:
                qx, qy, qz, qw = q.x / norm, q.y / norm, q.z / norm, q.w / norm
                r_mat = quat_to_rot_matrix(qx, qy, qz, qw)
                self.proj_gravity = (r_mat.T @ np.array([0.0, 0.0, -1.0])).astype(np.float32)

            gyro = msg.angular_velocity
            self.omega_base = np.array([gyro.x, gyro.y, gyro.z], dtype=np.float32)
            self.has_imu = True

        def cmd_vel_callback(self, msg: Twist):
            self.command = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)
            self.get_logger().info(f"Received /cmd_vel: vx={msg.linear.x:.2f}, wz={msg.angular.z:.2f}")

        def control_loop(self):
            if not self.has_joint_state or not self.has_imu:
                return

            if not self.logged_active:
                self.get_logger().info("Sensors online. Standing stable in nominal pose, awaiting /cmd_vel...")
                self.logged_active = True

            cmd_speed = float(np.linalg.norm(self.command[:2])) + abs(float(self.command[2]))
            if cmd_speed < 0.02:
                # Standby: hold rock-solid nominal stance until destination/velocity command arrives
                pos_msg = Float64MultiArray(data=[float(p) for p in self.q_nominal])
                self.pub_pos.publish(pos_msg)
                return

            self.step_count += 1
            leg_q = np.array([self.joint_positions[j] for j in LEG_JOINTS], dtype=np.float32)
            leg_dq = np.array([self.joint_velocities[j] for j in LEG_JOINTS], dtype=np.float32)
            leg_q_err = leg_q - self.leg_q_nominal

            if self.expected_obs_dim == 64:
                arm_q = np.array([self.joint_positions[j] for j in ARM_JOINTS], dtype=np.float32)
                prev_leg_act = self.prev_action[:12]
                cmd_vec = self.command
                phase = (self.step_count * self.dt) % 1.6 / 1.6
                phase_vec = np.array([
                    math.sin(2.0 * math.pi * phase),
                    math.cos(2.0 * math.pi * phase)
                ], dtype=np.float32)
                fsr_vec = np.zeros(4, dtype=np.float32)
                tof_val = np.float32(1.0)
                cam_vec = np.array([0.0, 0.0, 1.0], dtype=np.float32)
                power_vec = np.array([0.5, 0.2], dtype=np.float32)
                temp_vec = np.array([0.35, 0.35], dtype=np.float32)
                accel_z_impact = np.float32(0.0)

                obs = np.concatenate([
                    self.proj_gravity,       # 3
                    self.omega_base,         # 3
                    [accel_z_impact],        # 1
                    leg_q_err,               # 12
                    leg_dq,                  # 12
                    arm_q,                   # 4
                    prev_leg_act,            # 12
                    cmd_vec,                 # 3
                    phase_vec,               # 2
                    fsr_vec,                 # 4
                    [tof_val],               # 1
                    cam_vec,                 # 3
                    power_vec,               # 2
                    temp_vec,                # 2
                ]).reshape(1, 64).astype(np.float32)
            else:
                obs = np.concatenate([
                    self.proj_gravity,       # 3
                    self.omega_base,         # 3
                    leg_q_err,               # 12
                    leg_dq,                  # 12
                    self.prev_action[:12],   # 12
                    self.command,            # 3
                ]).reshape(1, self.expected_obs_dim).astype(np.float32)

            # ONNX Inference (< 0.3 ms)
            outputs = self.session.run(None, {self.input_name: obs})
            action = np.clip(outputs[0][0], -1.0, 1.0)
            self.prev_action = action.copy()

            # Map 12 leg actions to target angles
            target_leg_q = self.leg_q_nominal + action[:12] * self.action_scale

            full_positions = self.q_nominal.copy()
            for idx, joint_name in enumerate(LEG_JOINTS):
                j_idx = gait.JOINTS.index(joint_name)
                full_positions[j_idx] = float(target_leg_q[idx])

            # Map 4 arm swing actions if 16-dim action space
            if self.expected_act_dim >= 16:
                target_arm_q = self.arm_q_nominal + action[12:16] * (self.action_scale * 1.5)
                for idx, joint_name in enumerate(ARM_JOINTS):
                    j_idx = gait.JOINTS.index(joint_name)
                    full_positions[j_idx] = float(target_arm_q[idx])

            # 1. Publish directly to position commands (instant in MuJoCo bridge)
            pos_msg = Float64MultiArray(data=[float(p) for p in full_positions])
            self.pub_pos.publish(pos_msg)

            # 2. Also publish JointTrajectory for the trajectory topic (MuJoCo bridge)
            if self.pub_traj.get_subscription_count() > 0:
                traj = JointTrajectory(joint_names=gait.JOINTS)
                ns = int(round(self.dt * 1e9))
                pt = JointTrajectoryPoint(
                    positions=[float(p) for p in full_positions],
                    time_from_start=Duration(sec=ns // 10**9, nanosec=ns % 10**9)
                )
                traj.points.append(pt)
                self.pub_traj.publish(traj)

    return MicoRLNode()


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
