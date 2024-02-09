#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

#node inheritor
class MyNode(Node):

    def __init__(self):
        super().__init__("first_node")
        self.get_logger().info("Hello from ROS2")


def main(args=None):
    rclpy.init(args=args) #initialize ros2 communication
    node = MyNode() #instantiate a node (starts node)
    rclpy.spin(node) # .spin runs a node until it is manually killed

    rclpy.shutdown() # end ros2 comms

if __name__ == '__main__':
    main()
