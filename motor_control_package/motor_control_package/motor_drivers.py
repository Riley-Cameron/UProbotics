#!/usr/bin/env python3

import motor_control_package.motor_control_package.BLDC as BLDC
import Actuator as Actuator

from std_msgs.msg import Float64
import rclpy
from rclpy import Node
import RPi.GPIO as GPIO

class MotorDriver(Node):
    
    def __init__(self):
        self.motors = []

        self.motors.append(BLDC(topic='/motor/tread_left', pin=12, dir_pin=16, en_pin=20, freq=50, min_dc=0, max_dc=90, init_range=0))
        self.motors.append(BLDC(topic='/motor/tread_right', pin=13, dir_pin=6, en_pin=5, freq=50, min_dc=0, max_dc=90, init_range=0))
        self.get_logger().info("Initialized Drive Motors")

        self.motors.append(Actuator(topic='/motor/arm_angle_joint', pinA=23, pinB=24))
        self.get_logger().info("Initialized Arm Articulation")

        self.motors.append(Actuator(topic='/motor/bucket_angle_joint', pinA=7, pinB=8))
        self.get_logger().info("Initialized Bucket Articulation")

        for motor in self.motors:
            motor.start()


if __name__ == '__main__':
    md = MotorDriver()

    rclpy.spin()

    GPIO.cleanup()