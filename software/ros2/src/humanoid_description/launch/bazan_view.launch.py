"""Bazan in RViz2 without physics: kinematic playback of the bazan_gait demo (bazan_playback.py).
The robot is urdf/bazan_cad.urdf, generated from the CAD (hardware/mechanical/gen_urdf.py).

For physics use bazan_mujoco.launch.py (MuJoCo).
    ros2 launch humanoid_description bazan_view.launch.py                  # walk 8 steps, turn 60 deg, wave, loop
    ros2 launch humanoid_description bazan_view.launch.py motion:=stand    # hold the standing pose
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motion = LaunchConfiguration('motion')
    rviz = LaunchConfiguration('rviz')
    args = [
        DeclareLaunchArgument('motion', default_value='demo',
                              description='demo: loop walk / turn / wave, stand: hold the standing pose'),
        DeclareLaunchArgument('rviz', default_value='true', description='open RViz2 with rviz/bazan.rviz'),
    ]

    share = FindPackageShare('humanoid_description')
    robot_description = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
                 PathJoinSubstitution([share, 'urdf', 'bazan_cad.urdf'])]),
        value_type=str)

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_description}])

    playback = Node(package='humanoid_control', executable='bazan_playback.py', output='screen',
                    parameters=[{'robot_description': robot_description, 'motion': motion}])

    rviz2 = Node(package='rviz2', executable='rviz2', output='log', condition=IfCondition(rviz),
                 arguments=['-d', PathJoinSubstitution([share, 'rviz', 'bazan.rviz'])])

    return LaunchDescription(args + [rsp, playback, rviz2])
