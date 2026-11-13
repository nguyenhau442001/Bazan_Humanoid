#!/usr/bin/env python3
"""Programmatic manipulation and arm commanding node for Bazan Humanoid.

Demonstrates controlling Bazan's 3-DOF arms and 2-DOF head via MoveIt 2:
  - Named target goals (home, ready, wave, tuck)
  - Cartesian space reaching for manipulation
  - Synchronized head tracking and dual-arm gestures

Usage:
    ros2 run humanoid_moveit_config bazan_arm_commander.py --target wave
    ros2 run humanoid_moveit_config bazan_arm_commander.py --target ready
    ros2 run humanoid_moveit_config bazan_arm_commander.py --target home
"""
import argparse
import sys

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


NAMED_POSES = {
    "left_arm": {
        "home": {"l_shoulder_pitch": 0.0, "l_shoulder_roll": 0.0, "l_elbow": 0.0},
        "ready": {"l_shoulder_pitch": 0.5236, "l_shoulder_roll": 0.2618, "l_elbow": -0.7854},
        "reach_forward": {"l_shoulder_pitch": -1.5708, "l_shoulder_roll": 0.0, "l_elbow": 0.0},
        "wave": {"l_shoulder_pitch": -2.0944, "l_shoulder_roll": 1.0472, "l_elbow": -1.2217},
        "tuck": {"l_shoulder_pitch": -0.3491, "l_shoulder_roll": 0.1745, "l_elbow": -1.5708},
    },
    "right_arm": {
        "home": {"r_shoulder_pitch": 0.0, "r_shoulder_roll": 0.0, "r_elbow": 0.0},
        "ready": {"r_shoulder_pitch": 0.5236, "r_shoulder_roll": -0.2618, "r_elbow": -0.7854},
        "reach_forward": {"r_shoulder_pitch": -1.5708, "r_shoulder_roll": 0.0, "r_elbow": 0.0},
        "wave": {"r_shoulder_pitch": -2.0944, "r_shoulder_roll": -1.0472, "r_elbow": -1.2217},
        "tuck": {"r_shoulder_pitch": -0.3491, "r_shoulder_roll": -0.1745, "r_elbow": -1.5708},
    },
    "head": {
        "center": {"neck_yaw": 0.0, "head_pitch": 0.0},
        "look_down": {"neck_yaw": 0.0, "head_pitch": -0.3491},
        "look_up": {"neck_yaw": 0.0, "head_pitch": 0.3491},
        "look_left": {"neck_yaw": 0.7854, "head_pitch": 0.0},
        "look_right": {"neck_yaw": -0.7854, "head_pitch": 0.0},
    },
}


class BazanArmCommander(Node):
    """Action client interface for sending motion goals to MoveIt 2 move_group."""

    def __init__(self):
        super().__init__("bazan_arm_commander")
        self.client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Connecting to MoveIt 2 move_action server...")

    def wait_for_server(self, timeout_sec=10.0):
        return self.client.wait_for_server(timeout_sec=timeout_sec)

    def move_to_named_pose(self, group_name="left_arm", target_name="ready"):
        """Plan and execute a named joint target pose."""
        if group_name not in NAMED_POSES or target_name not in NAMED_POSES[group_name]:
            self.get_logger().error(f"Unknown target '{target_name}' for group '{group_name}'")
            return False

        joint_dict = NAMED_POSES[group_name][target_name]
        self.get_logger().info(f"Planning {group_name} -> '{target_name}' ({joint_dict})")

        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.allowed_planning_time = 2.0
        goal.request.max_velocity_scaling_factor = 0.8
        goal.request.max_acceleration_scaling_factor = 0.8

        constraints = Constraints()
        for joint_name, position in joint_dict.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(position)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Motion goal was rejected by move_group")
            return False

        self.get_logger().info("Goal accepted. Executing trajectory...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result

        if res.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f"SUCCESS: {group_name} reached '{target_name}'")
            return True
        else:
            self.get_logger().warn(f"Motion execution finished with error code: {res.error_code.val}")
            return False


def main():
    parser = argparse.ArgumentParser(description="Bazan Arm Commander CLI")
    valid_targets = [
        "home", "ready", "reach_forward", "wave", "tuck",
        "center", "look_down", "look_up", "look_left", "look_right"
    ]
    parser.add_argument("--group", default="left_arm", choices=["left_arm", "right_arm", "head"])
    parser.add_argument("--target", default="ready", choices=valid_targets)
    args = parser.parse_args()

    rclpy.init()
    node = BazanArmCommander()

    if not node.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("MoveGroup action server not available. Ensure move_group is running.")
        sys.exit(1)

    node.move_to_named_pose(args.group, args.target)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
