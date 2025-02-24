from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='srv_pkg',
            executable='srv_node',
            output='screen'),
    ])


































 