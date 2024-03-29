#!/usr/bin/env python3
from .BLDC import BLDC
import rclpy
import RPi.GPIO as GPIO

def main(args=None):
    rclpy.init(args=args) #initialize ros2 communication

    node = BLDC(topic='/motor/tread_right', node='right_tread', pin=13, dir_pin=6, en_pin=5, freq=50, min_dc=0, max_dc=90, init_range=0) #instantiate a node (starts node)
    node.start()

    rclpy.spin(node) # .spin runs a node until it is manually killed

    GPIO.cleanup()
    rclpy.shutdown() # end ros2 comms

if __name__ == '__main__':
    main()