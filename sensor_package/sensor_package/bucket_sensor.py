#!/usr/bin/env python3
from .ActuatorSensor import ActuatorSensor
import rclpy


def main(args=None):
    rclpy.init(args=args) #initialize ros2 communication

    node = ActuatorSensor(signal_topic='/motor/bucket_angle_joint', pos_topic='/sensor/bucket_joint', node='bucket_sensor', extension_time=22500) #instantiate a node (starts node)

    try:
        rclpy.spin(node) # .spin runs a node until it is manually killed
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown() # end ros2 comms
        print("bucket sensor killed successfully")

if __name__ == '__main__':
    main()