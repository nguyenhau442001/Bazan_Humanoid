"""Bazan in MuJoCo with physics, viewed in RViz2.

Native on macOS (ros2_jazzy conda env) or in the Ubuntu VM:
    ros2 launch humanoid_description bazan_mujoco.launch.py                     # stand, RViz2
    ros2 launch humanoid_description bazan_mujoco.launch.py gait:=true          # walk 8 steps, turn 60 deg, wave
    ros2 launch humanoid_description bazan_mujoco.launch.py gait:=true loop:=true viewer:=true   # nonstop
    ros2 launch humanoid_description bazan_mujoco.launch.py viewer:=true        # also the MuJoCo viewer
    ros2 launch humanoid_description bazan_mujoco.launch.py rviz:=false look:=boxes
"""
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# the MuJoCo viewer needs the main thread on macOS: run the bridge with mjpython there
VIEWER_PYTHON = 'mjpython' if sys.platform == 'darwin' else ''


def generate_launch_description():
    look = LaunchConfiguration('look')
    mass = LaunchConfiguration('mass')
    gait = LaunchConfiguration('gait')
    rl = LaunchConfiguration('rl')
    loop = LaunchConfiguration('loop')
    rviz = LaunchConfiguration('rviz')
    viewer = LaunchConfiguration('viewer')
    args = [
        DeclareLaunchArgument('gait', default_value='false', description='run the bazan_gait demo'),
        DeclareLaunchArgument('rl', default_value='false', description='run the trained ONNX RL locomotion controller'),
        DeclareLaunchArgument('loop', default_value='false',
                              description='repeat the demo nonstop (each pass turns another 60 deg)'),
        DeclareLaunchArgument('rviz', default_value='true', description='open RViz2 with rviz/bazan.rviz'),
        DeclareLaunchArgument('viewer', default_value='false', description='open the MuJoCo viewer'),
        DeclareLaunchArgument('look', default_value='bazan',
                              description='bazan: Blender shell meshes, boxes: servo boxes'),
        DeclareLaunchArgument('mass', default_value='sim',
                              description='sim: masses the gait was tuned on, blender: shell mass budget'),
    ]

    share = FindPackageShare('humanoid_description')
    robot_description = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
                 PathJoinSubstitution([share, 'urdf', 'bazan.urdf.xacro']),
                 ' look:=', look, ' mass:=', mass]),
        value_type=str)

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    bridge = Node(package='humanoid_control', executable='bazan_mujoco_bridge.py', output='screen',
                  prefix=PythonExpression(["'", VIEWER_PYTHON, "' if '", viewer, "' == 'true' else ''"]),
                  parameters=[{'robot_description': robot_description,
                               'viewer': ParameterValue(viewer, value_type=bool)}])

    walk = Node(package='humanoid_control', executable='bazan_gait.py', output='screen', condition=IfCondition(gait),
                parameters=[{'use_sim_time': True, 'loop': ParameterValue(loop, value_type=bool)}])

    rl_node = Node(package='humanoid_control', executable='bazan_rl_node.py',
                   output='screen', condition=IfCondition(rl),
                   parameters=[{'use_sim_time': True}])

    rviz2 = Node(package='rviz2', executable='rviz2', output='log', condition=IfCondition(rviz),
                 arguments=['-d', PathJoinSubstitution([share, 'rviz', 'bazan.rviz'])],
                 parameters=[{'use_sim_time': True}])

    return LaunchDescription(args + [rsp, bridge, walk, rl_node, rviz2])
