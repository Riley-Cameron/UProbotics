from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription([
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
        Node(
            package='motor_control_package',
            executable='arm_joint', 
            name='aj'
        ),
        Node(
            package='motor_control_package',
            executable='bucket_joint', 
            name='bj' # ;)
        ),
        # Add more nodes as needed
    ])

    return ld