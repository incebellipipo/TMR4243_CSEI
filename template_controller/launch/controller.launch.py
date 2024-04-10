#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from tmr4243_utilities.utilities import anon


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='template_controller',
            executable='controller_node.py',
            name=f'{anon()}controller'
        ),
    ])