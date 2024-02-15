from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='template_thrust_allocation',
            executable='thrust_allocation_node.py',
            name='thrust_allocation'
        ),
    ])