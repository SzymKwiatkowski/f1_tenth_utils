from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    waypoints_file = get_package_share_directory('pp_controller') + "/config/waypoints.csv"
    return LaunchDescription([
        Node(
            package='pp_controller',
            namespace='pp_controller',
            executable='pure_pursuit_f1_tenth_controller',
            parameters=[{
            'waypoints_file': waypoints_file
            }],
        )
    ])
