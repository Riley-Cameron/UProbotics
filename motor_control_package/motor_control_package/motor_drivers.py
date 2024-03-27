from MotorListener import MotorListener
from BLDC import BLDC
from Actuator import Actuator
import rclpy
import RPi.GPIO as GPIO

def main(args=None):
    rclpy.init(args=args)
    
    motors = []

    #Create motor instances
    motors.append(BLDC(topic='/motor/tread_left', node='left_tread', pin=12, dir_pin=16, en_pin=20, freq=50, min_dc=0, max_dc=90, init_range=0))
    motors.append(BLDC(topic='/motor/tread_right', node='right_tread', pin=13, dir_pin=6, en_pin=5, freq=50, min_dc=0, max_dc=90, init_range=0))
    motors.append(Actuator(topic='/motor/arm_angle_joint', node='arm_joint', pinA=23, pinB=24))
    motors.append(Actuator(topic='/motor/bucket_angle_joint', node='bucket_joint', pinA=7, pinB=8))

    #Start all motor threads
    for motor in motors:
        motor.start()

    #spin each motor's event loop
    try:
        for motor in motors:
            rclpy.spin(motor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Exception occurred while spinning {motor.node} node: {e}")

    #stop and join all motor threads
    for motor in motors:
        motor.stop()
        motor.join()

    #Clean up GPIO and shutdown ROS 2
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
