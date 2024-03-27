#!/usr/bin/env python3
from MotorListener import MotorListener
import RPi.GPIO as GPIO
import rclpy

class Actuator(MotorListener):

    STOPPED = 0
    EXTEND = 1
    RETRACT = 2

    def __init__(self, topic, node, pinA, pinB):
        """
        A PWM class that sets RPi pin to specified duty cycle and freqency
            
        Dependencies:
            RPi.GPIO
        Parameters:
            topic: name of ROS topic to subscribe to
            pinA: Board pin for controlling relay A (BCM)
            pinB: Board pin for controlling relay B (BCM)
        """
        super().__init__(topic, node)
        self.state = self.STOPPED
        self.pinA = pinA
        self.pinB = pinB
        try:
            GPIO.setmode(GPIO.BCM)
        except Exception:
            print('GPIO failure')
        try:
            GPIO.setup(pinA, GPIO.OUT, initial = GPIO.LOW)
        except Exception:
            print('PinA setup failure')
        try:
            GPIO.setup(pinB, GPIO.OUT, initial = GPIO.LOW)
        except Exception:
            print("PinB setup failure") 

        self.get_logger().info("Actuator Node Initialized")
        
    def setState(self, data):
        if (data == self.STOPPED):
            self.state = self.STOPPED
        elif (data == self.EXTEND):
            self.state = self.EXTEND
        elif (data == self.RETRACT):
            self.state = self.RETRACT
        else:
            self.state = self.STOPPED
            print("Invalid State Recieved, Actuator Stopped!")

        self.setPins()

    def setPins(self):
        if (self.state == self.STOPPED):
            GPIO.output(self.pinA, GPIO.LOW)
            GPIO.output(self.pinB, GPIO.LOW)
        elif (self.state == self.EXTEND):
            GPIO.output(self.pinA, GPIO.HIGH)
            GPIO.output(self.pinB, GPIO.LOW)
        elif (self.state == self.RETRACT):
            GPIO.output(self.pinA, GPIO.LOW)
            GPIO.output(self.pinB, GPIO.HIGH)     

    def disable(self):
        self.setState(self.STOPPED)

    def update(self, data):
        self.setState(data)

    def loop(self):
        pass

    def on_exit(self):
        pass