from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='act_client_pkg',
            executable='action_client_node',
            output='screen'),
    ])