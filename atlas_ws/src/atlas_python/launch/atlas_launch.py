from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.parameter_descriptions import Parameter
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
            default_value=0,
            description="Camera number to use for object detection",
        ),
        Node(
            package='camera_ros',
            executable='camera_node',
            parameters=[
                Parameter("camera", LaunchConfiguration("camera")),
                Parameter("height", 480),
                Parameter("width", 640)
            ],
        ),
        Node(
            package='atlas_python',
            executable='object_detection',
            parameters=[
                Parameter("model_name", LaunchConfiguration("model_name"))
            ],
        ),
    ])