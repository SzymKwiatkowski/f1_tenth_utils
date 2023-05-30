from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoints_publisher',
            namespace='waypoints_publisher',
            executable='waypoints_publisher',
            name='waypoints_publisher'
        )
    ])
