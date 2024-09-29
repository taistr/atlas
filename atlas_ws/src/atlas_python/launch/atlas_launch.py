from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_name",
            default_value="29_08_640px.onnx",
            description="Name of the model to use for object detection",
        ),
        DeclareLaunchArgument(
            "camera",
            default_value="0",  # Changed to a string to match LaunchConfiguration expectations
            description="Camera number to use for object detection",
        ),
        Node(
            package='camera_ros',
            executable='camera_node',
            parameters=[
                {"camera": LaunchConfiguration("camera")},
                {"height": ParameterValue(480, value_type=int)},  # Wrapped in ParameterValue
                {"width": ParameterValue(640, value_type=int)},   # Wrapped in ParameterValue
            ],
        ),
        Node(
            package='atlas_python',
            executable='object_detection',
            parameters=[
                {"model_name": LaunchConfiguration("model_name")}
            ],
        ),
        Node(
            package='atlas_python',
            executable='serial_comms',
        ),
    ])
