#!/usr/bin/env python3
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from threading import Thread

class MotorListener(ABC, Thread, Node):

    """
    Parameters:
        topic: String for ROS topic name
        updated: True if we have a new value to pass to motor, else False
        data: Last message from the topic stored
    """
    def __init__(self, topic: str):
        Thread.__init__()
        ABC.__init__()
        Node.init("node"+topic)
        self.sub = self.create_subscription(Float64, topic, self.topic_callback, 10)
        self.updated = True
        self.data = None
        self.stop = False


    def topic_callback(self, msg: Float64):
        if self.data == msg.data:
            return
        self.data = msg.data # Unpack std_msgs.Float64 to float
        self.updated = False


    """
    Gets called only when we have a new message for motor
    """
    @abstractmethod
    def update(self, data):
        pass

    """
    Gets called on every cycle of loop
    """
    @abstractmethod
    def loop(self):
        pass

    """
    Gets called when the thread is finished
    """
    @abstractmethod
    def on_exit(self):
        pass
    
    def run(self):
        while not rclpy.is_shutdown() and not self.stop:
            if not self.updated:
                self.updated = True
                self.update(self.data)
            self.loop()
        self.on_exit()