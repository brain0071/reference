from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='reference',
            executable='reference_node',
            name='reference_node',
            output='screen',
            emulate_tty=True,
        ),
    ])