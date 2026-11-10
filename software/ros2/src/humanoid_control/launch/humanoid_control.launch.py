from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['lower_body_trajectory_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['upper_body_trajectory_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='humanoid_control',
            executable='joint_trajectory_bridge_node',
            name='joint_trajectory_bridge',
            output='screen',
        )
    ])
