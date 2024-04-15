from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='teleoperation_package',
            executable='teleop_node',  
            name='teleop_node'
        ),
        Node(
            package='base_control_package',
            executable='base_control', 
            name='base_control'
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view', 
            name='image_viewer1'
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view', 
            name='image_viewer2'
        ),
        # Add more nodes as needed
    ])

    return ld