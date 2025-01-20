#!/usr/bin/env python3

import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from tmr4243_utilities.utilities import anon

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('template_observer'),
        'config',
        'param.yaml'
    )

    arg_task = launch.actions.DeclareLaunchArgument(
        'task',
        default_value=launch.substitutions.TextSubstitution(text='luenberger'),
        description='Observer task type.',
        choices=['luenberger', 'deadreckoning']
    )

    node_observer = launch_ros.actions.Node(
        package='template_observer',
        executable='observer_node.py',
        name=f'{anon()}observer',
        parameters=[
            config,
            {'task': launch.substitutions.LaunchConfiguration('task')}
            ],
        output='screen'
    )

    return launch.LaunchDescription([
        arg_task,
        node_observer
    ])
