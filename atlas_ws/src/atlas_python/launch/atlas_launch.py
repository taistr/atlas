from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='atlas_python',
            executable='planner',
        ),
        Node(
            package='atlas_python',
            executable='camera',
        ),
        Node(
            package='atlas_python',
            executable='object_detection',
        ),
        Node(
            package='atlas_python',
            executable='serial_comms',
        ),
        Node(
            package='atlas_python',
            executable='motor_driver',
        )
    ])