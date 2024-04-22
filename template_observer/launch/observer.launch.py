#!/usr/bin/env python3

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os
from tmr4243_utilities.utilities import anon

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('template_observer'),
        'config',
        'yaml.yaml'
    )

    arg_task = launch.actions.DeclareLaunchArgument(
        'task',
        default_value=launch.substitutions.TextSubstitution(text='luenberg'),
        description='Controller task type. Choose between PD_FF_controller, PID_controller, backstepping_controller.',
        choices=['PD_FF_controller', 'PID_controller', 'backstepping_controller']
    )

    arg_param_file = launch.actions.DeclareLaunchArgument(
        'param_file',
        default_value=launch.substitutions.PathJoinSubstitution(
            [launch_ros.substitutions.FindPackageShare('template_controller'), 'config', 'param.yaml']
        )
    )

    node_observer = launch_ros.actions.Node(
        package='template_observer',
        executable='observer_node.py',
        name=f'{anon()}observer',
        parameters=[config],
        output='screen'
    )

    return launch.LaunchDescription([
        node_observer
    ])
