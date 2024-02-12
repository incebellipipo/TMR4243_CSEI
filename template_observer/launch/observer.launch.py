from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='template_observer',
            executable='observer.py',
            name='observer'
        ),
    ])