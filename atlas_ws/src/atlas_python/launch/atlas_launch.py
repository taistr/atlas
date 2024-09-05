from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
        ),
        Node(
            package='atlas_python',
            executable='hello',
        ),
        Node(
            package='atlas_python',
            executable='object_detection',
        ),
    ])