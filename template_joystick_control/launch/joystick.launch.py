import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():

    arg_task = launch.actions.DeclareLaunchArgument(
        'task',
        default_value=launch.substitutions.TextSubstitution(text='simple'),
        description='Joystick control task type. Choose between simple, basin, and body.',
        choices=['simple', 'basin', 'body']
    )

    node_joystick_control = launch_ros.actions.Node(
        package='template_joystick_control',
        executable='joystick_control_node.py',
        name='joystick_control',
        parameters=[
                {'task': launch.substitutions.LaunchConfiguration('task')},
        ]
    )
    node_joy = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
    )

    return launch.LaunchDescription([
        arg_task,
        node_joystick_control,
        node_joy
    ])
