#!/usr/bin/env python3

import PWM

from std_msgs.msg import Float64
import rclpy
from rclpy import Node
import RPi.GPIO as GPIO

class MotorDriver(Node):
    
    def __init__(self):
        self.motors = []

        self.motors.append(PWM(topic='/motor/tread_left', pin=16, freq=1000, min_dc=0, max_dc=90, init_range=0))
        self.motors.append(PWM(topic='/motor/tread_right', pin=16, freq=1000, min_dc=0, max_dc=90, init_range=0))
        self.get_logger().info("Initialized Drive Motors")

        self.motors.append(PWM(topic='/motor/arm_angle_joint', pin=16, freq=50, min_dc=0, max_dc=100, init_range=0))
        self.get_logger().info("Initialized Arm Articulation")

        self.motors.append(PWM(topic='/motor/bucket_angle_joint', pin=16, freq=50, min_dc=0, max_dc=100, init_range=0))
        self.get_logger().info("Initialized Bucket Articulation")

        for motor in self.motors:
            motor.start()


if __name__ == '__main__':
    md = MotorDriver()

    rclpy.spin()

    GPIO.cleanup()