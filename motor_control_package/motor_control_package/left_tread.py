#!/usr/bin/env python3
from .BLDC import BLDC
import rclpy
import RPi.GPIO as GPIO

def main(args=None):
    rclpy.init(args=args) #initialize ros2 communication

    node = BLDC(topic='/motor/tread_left', node='left_tread', pin=12, dir_pin=16, en_pin=20, freq=50, min_dc=0, max_dc=90, init_range=0) #instantiate a node (starts node)
    node.start()

    try:
        rclpy.spin(node) # .spin runs a node until it is manually killed
    except KeyboardInterrupt:
        print("Kill signal exception caught")
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown() # end ros2 comms
        print("left tread killed successfully")

if __name__ == '__main__':
    main()