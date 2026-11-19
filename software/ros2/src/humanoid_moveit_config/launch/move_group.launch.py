"""Launch move_group node for Bazan Humanoid manipulation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(file_path):
    try:
        with open(file_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock if true"
    )

    desc_share = FindPackageShare("humanoid_description")
    moveit_share = FindPackageShare("humanoid_moveit_config")

    # Robot Description (URDF from xacro)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([desc_share, "urdf", "bazan_cad.urdf"]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Robot Description Semantic (SRDF)
    srdf_path = PathJoinSubstitution([moveit_share, "config", "bazan.srdf"])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(
        Command(["cat ", srdf_path]), value_type=str
    )}

    # Kinematics config
    kinematics_yaml = PathJoinSubstitution([moveit_share, "config", "kinematics.yaml"])

    # Joint Limits
    joint_limits_yaml = PathJoinSubstitution([moveit_share, "config", "joint_limits.yaml"])

    # Planning Pipelines (OMPL)
    ompl_yaml = PathJoinSubstitution([moveit_share, "config", "ompl_planning.yaml"])
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/ResolveConstraintFrames "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }

    # Trajectory Execution / Controllers
    moveit_controllers = PathJoinSubstitution([moveit_share, "config", "moveit_controllers.yaml"])
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_planning_pipeline_config,
            ompl_yaml,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        move_group_node,
    ])
