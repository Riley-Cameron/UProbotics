#!/usr/bin/env python3
from MotorListener import MotorListener
from BLDC import BLDC
from Actuator import Actuator
import rclpy
import RPi.GPIO as GPIO

        
def main(args=None):
    rclpy.init(args=args)
    
    motors = []

    motors.append(BLDC(topic='/motor/tread_left', node='left_tread', pin=12, dir_pin=16, en_pin=20, freq=50, min_dc=0, max_dc=90, init_range=0))
    motors.append(BLDC(topic='/motor/tread_right', node='right_tread', pin=13, dir_pin=6, en_pin=5, freq=50, min_dc=0, max_dc=90, init_range=0))

    motors.append(Actuator(topic='/motor/arm_angle_joint', node='arm_joint', pinA=23, pinB=24))

    motors.append(Actuator(topic='/motor/bucket_angle_joint', node='bucket_joint', pinA=7, pinB=8))

    for motor in motors:
        motor.start()

    for motor in motors:
        try:
            rclpy.spin(motor)
        except KeyboardInterrupt:
            pass

    for motor in motors:
        motor.stop()

    for motor in motors:
        motor.join()

    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
