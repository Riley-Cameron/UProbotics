#!/usr/bin/env python3
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MotorListener(ABC, Node):

    """
    Parameters:
        topic: String for ROS topic name
        node: String for ROS node name
        updated: True if we have a new value to pass to motor, else False
        data: Last message from the topic stored
    """
    def __init__(self, topic: str, node: str):
        ABC.__init__(self)
        Node.__init__(self, node)
        self.sub = self.create_subscription(Float64, topic, self.topic_callback, 10)
        self.data = 0.0
        self.stop = False


    def topic_callback(self, msg: Float64):
        if self.data == msg.data:
            return
        self.data = msg.data # Unpack std_msgs.Float64 to float
        self.update(self.data)


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
    