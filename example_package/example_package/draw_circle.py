#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

#node inheritor
class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.recieve_pose, 10)
        self.get_logger().info("Drawing Circle:")
        self.create_timer(1, self.send_vel)

    def send_vel(self):
        msg = Twist()
        msg.linear.x = -1.5
        msg.angular.z = 0.77
        self.cmd_vel_pub_.publish(msg)

    def recieve_pose(self, msg: Pose):
        self.get_logger().info("Turtle direction: " + str(msg.theta))


def main(args=None):
    rclpy.init(args=args) #initialize ros2 communication
    node = DrawCircleNode() #instantiate a node (starts node)
    rclpy.spin(node) # .spin runs a node until it is manually killed
    rclpy.shutdown() # end ros2 comms
