from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='sensor_package',
            executable='arm_sensor',  
            name='arm_sensor'
        ),
        Node(
            package='sensor_package',
            executable='bucket_sensor', 
            name='bucket_sensor'
        ),
        # Add more nodes as needed
    ])

    return ld