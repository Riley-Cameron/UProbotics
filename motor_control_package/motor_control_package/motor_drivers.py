from threading import Thread
from MotorListener import MotorListener
from BLDC import BLDC
from Actuator import Actuator
import rclpy
import RPi.GPIO as GPIO

class NodeThread(Thread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)

def main(args=None):
    rclpy.init(args=args)
    
    motors = []

    # Create motor instances
    motors.append(BLDC(topic='/motor/tread_left', node='left_tread', pin=12, dir_pin=16, en_pin=20, freq=50, min_dc=0, max_dc=90, init_range=0))
    motors.append(BLDC(topic='/motor/tread_right', node='right_tread', pin=13, dir_pin=6, en_pin=5, freq=50, min_dc=0, max_dc=90, init_range=0))
    motors.append(Actuator(topic='/motor/arm_angle_joint', node='arm_joint', pinA=23, pinB=24))
    motors.append(Actuator(topic='/motor/bucket_angle_joint', node='bucket_joint', pinA=7, pinB=8))

    # Create and start threads for each node
    threads = [NodeThread(motor) for motor in motors]
    for thread in threads:
        thread.start()

    # Wait for all threads to finish
    for thread in threads:
        thread.join()

    # Clean up GPIO and shutdown ROS 2
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

