"""Complete interactive MoveIt 2 demo for Bazan Humanoid manipulation in RViz2."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_share = FindPackageShare("humanoid_description")
    moveit_share = FindPackageShare("humanoid_moveit_config")

    use_gui = LaunchConfiguration("use_gui")
    declare_use_gui = DeclareLaunchArgument(
        "use_gui", default_value="true", description="Start joint_state_publisher_gui if true"
    )

    # Robot Description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([desc_share, "urdf", "bazan_cad.urdf"]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Joint State Publisher GUI (allows posing joints or visualizing execution)
    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(use_gui),
    )

    # Include move_group launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_share, "launch", "move_group.launch.py"])
        )
    )

    # SRDF for RViz MotionPlanning plugin
    srdf_path = PathJoinSubstitution([moveit_share, "config", "bazan.srdf"])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(
        Command(["cat ", srdf_path]), value_type=str
    )}

    kinematics_yaml = PathJoinSubstitution([moveit_share, "config", "kinematics.yaml"])

    # RViz2 with pre-configured MoveIt display
    rviz_config_path = PathJoinSubstitution([moveit_share, "config", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )

    # Static transform world -> base_link
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0.115", "0", "0", "0", "world", "base_link"],
    )

    return LaunchDescription([
        declare_use_gui,
        static_tf,
        rsp_node,
        jsp_gui_node,
        move_group_launch,
        rviz_node,
    ])
