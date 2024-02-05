from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='template_joystick_control',
            executable='joystick_control_node.py',
            name='joystick_control'
        ),
        Node(
            package='joy',
            executable='joy_node',
        ),
    ])