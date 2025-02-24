from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='act_server_pkg',
            executable='act_server_node',
            output='screen'),
    ])