import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():

    arg_l1 = launch.actions.DeclareLaunchArgument(
        'L1',
        default_value=launch.substitutions.PythonExpression('1.0'),
        description='L1 gain',
    )
    arg_l2 = launch.actions.DeclareLaunchArgument(
        'L2',
        default_value=launch.substitutions.PythonExpression('1.0'),
        description='L2 gain',
    )
    arg_l3 = launch.actions.DeclareLaunchArgument(
        'L3',
        default_value=launch.substitutions.PythonExpression('1.0'),
        description='L3 gain',
    )

    node_observer = launch_ros.actions.Node(
        package='template_observer',
        executable='observer_node.py',
        name='observer',
        parameters=[
            {'L1': launch.substitutions.LaunchConfiguration('L1')},
            {'L2': launch.substitutions.LaunchConfiguration('L2')},
            {'L3': launch.substitutions.LaunchConfiguration('L3')},
        ]
    )

    return launch.LaunchDescription([
        arg_l1,
        arg_l2,
        arg_l3,
        node_observer
    ])
