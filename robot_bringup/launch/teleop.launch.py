from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='teleoperation_package',
            executable='teleop_node',  
            name='tn'
        ),
        Node(
            package='base_control_package',
            executable='base_control', 
            name='bc'
        ),
        # Add more nodes as needed
    ])

    return ld