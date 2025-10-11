from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_id_checker',
            executable='pose_publisher',
            name='pose_publisher',
            parameters=[{'target_id': 7}],
            output='screen'
        )
    ])