from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='motor_control_package',
            executable='left_tread',  
            name='left_tread'
        ),
        Node(
            package='motor_control_package',
            executable='right_tread', 
            name='right_tread'
        ),
        Node(
            package='motor_control_package',
            executable='arm_joint', 
            name='arm_joint'
        ),
        Node(
            package='motor_control_package',
            executable='bucket_joint', 
            name='bucket_joint'
        ),
        Node(
            package='sensor_package',
            executable='camera1', 
            name='camera1'
        ),
        Node(
            package='sensor_package',
            executable='camera2', 
            name='camera2'
        ),
        # Add more nodes as needed
    ])

    return ld