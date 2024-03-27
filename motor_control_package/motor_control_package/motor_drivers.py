#!/usr/bin/env python3

from BLDC import BLDC
from Actuator import Actuator

from std_msgs.msg import Float64
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO

class MotorDriver(Node):
    
    def __init__(self):
        super().__init__("motor_drivers")
        self.motors = []

        self.motors.append(BLDC(topic='/motor/tread_left', node='left_tread', pin=12, dir_pin=16, en_pin=20, freq=50, min_dc=0, max_dc=90, init_range=0))
        self.motors.append(BLDC(topic='/motor/tread_right', node='right_tread', pin=13, dir_pin=6, en_pin=5, freq=50, min_dc=0, max_dc=90, init_range=0))
        self.get_logger().info("Initialized Drive Motors")

        self.motors.append(Actuator(topic='/motor/arm_angle_joint', node='arm_joint', pinA=23, pinB=24))
        self.get_logger().info("Initialized Arm Articulation")

        self.motors.append(Actuator(topic='/motor/bucket_angle_joint', node='bucket_joint', pinA=7, pinB=8))
        self.get_logger().info("Initialized Bucket Articulation")

        for motor in self.motors:
            motor.start()

def main(args=None):
    rclpy.init(args=args)
    md = MotorDriver()

    md.motors[0].spin()

    try:
        rclpy.spin(md)
    except KeyboardInterrupt:
        pass

    for motor in md.motors:
        motor.destroy_node()

    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
