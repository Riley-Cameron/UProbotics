#!/usr/bin/env python3
from .Actuator import Actuator
import rclpy
import RPi.GPIO as GPIO

def main(args=None):
    rclpy.init(args=args) #initialize ros2 communication

    node = Actuator(topic='/motor/arm_angle_joint', node='arm_joint', pinA=15, pinB=14) #instantiate a node (starts node)

    try:
        rclpy.spin(node) # .spin runs a node until it is manually killed
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        rclpy.shutdown() # end ros2 comms
        print("arm joint killed successfully")

if __name__ == '__main__':
    main()