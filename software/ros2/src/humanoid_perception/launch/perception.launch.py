from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_perception',
            executable='camera_subscriber_node',
            name='camera_subscriber_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='humanoid_perception',
            executable='onnx_inference_node',
            name='onnx_inference_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'model_path': 'models/yolov8_humanoid.onnx'},
                {'confidence_threshold': 0.45},
                {'use_cuda': True}
            ]
        )
    ])
