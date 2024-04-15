#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from tmr4243_utilities.utilities import anon


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='ros2_bag_to_csv',
            executable='ros2_bag_to_csv_node.py',
            name=f'{anon()}ros2_bag',
            output='screen'
        ),
    ])