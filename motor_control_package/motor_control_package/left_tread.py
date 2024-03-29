#!/usr/bin/env python3
from .BLDC import BLDC
import rclpy
import RPi.GPIO as GPIO
import signal
import sys

def sigint_handler(sig, frame):
    print("Keyboard interrupt detected. Cleaning up...")
    # Perform cleanup operations here
    GPIO.cleanup()
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args) #initialize ros2 communication

    node = BLDC(topic='/motor/tread_left', node='left_tread', pin=12, dir_pin=16, en_pin=20, freq=50, min_dc=0, max_dc=90, init_range=0) #instantiate a node (starts node)
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        rclpy.shutdown() # end ros2 comms
        print("left tread killed successfully")

if __name__ == '__main__':
    main()