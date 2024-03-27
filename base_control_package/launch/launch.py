from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_control_package',
            executable='left_tread',  
            name='lt'
        ),
        Node(
            package='motor_control_package',
            executable='right_tread', 
            name='rt'
        ),
        # Add more nodes as needed
    ])
