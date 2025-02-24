# Description: Launch file for sub_node
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sub_pkg',
            executable='sub_node',
            name='sub_node'
        ),
    ])
