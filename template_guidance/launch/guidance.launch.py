from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='template_guidance',
            executable='guidance_node.py',
            name='guidance'
        ),
    ])