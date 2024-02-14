#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

#node inheritor
class MyNode(Node):

    def __init__(self):
        super().__init__("first_node")
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hi " + str(self.counter_))
        self.counter_ += 1


def main(args=None):
    rclpy.init(args=args) #initialize ros2 communication
    node = MyNode() #instantiate a node (starts node)
    rclpy.spin(node) # .spin runs a node until it is manually killed

    rclpy.shutdown() # end ros2 comms

if __name__ == '__main__':
    main()
