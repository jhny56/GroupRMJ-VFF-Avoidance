import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='avoidance',
            executable='avoidance_node',
            name='avoidance_node',
            output='screen',
            parameters=[
                # Add any parameters here if needed
            ],
            remappings=[
                # Add any topic remappings here if needed
            ]
        ),
    ])
