#!/usr/bin/env python3
from Actuator import Actuator
import rclpy
import RPi.GPIO as GPIO

def main(args=None):
    rclpy.init(args=args) #initialize ros2 communication

    node = Actuator(topic='/motor/arm_angle_joint', node='arm_joint', pinA=23, pinB=24) #instantiate a node (starts node)
    node.start()

    rclpy.spin(node) # .spin runs a node until it is manually killed

    GPIO.cleanup()
    rclpy.shutdown() # end ros2 comms

if __name__ == '__main__':
    main()