#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int16

class BaseControl(Node):

    def __init__(self):
        super().__init__("base_control")
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd/velocity", self.recieve_vel, 10)
        self.cmd_arm_sub = self.create_subscription(Int16, "/cmd/arm_state", self.recieve_arm_state, 10)
        self.cmd_bucket_sub = self.create_subscription(Int16, "/cmd/bucket_state", self.recieve_bucket_state, 10)

        self.left_tread_pub = self.create_publisher(Float64, "/motor/tread_left", 10)
        self.right_tread_pub = self.create_publisher(Float64, "/motor/tread_right", 10)
        self.arm_state_pub = self.create_publisher(Float64, "/motor/arm_state", 10)
        self.bucket_state_pub = self.create_publisher(Float64, "/motor/bucket_state", 10)

        self.last_cmd_vel = Twist()
        self.last_cmd_arm = Int16()
        self.last_cmd_bucket = Int16()
        self.get_logger().info("Base Control Initialized")

    def recieve_vel(self, msg: Twist):
        #only send new values when necessary
        if (msg.angular.z != self.last_cmd_vel.angular.z or abs(msg.linear.x - self.last_cmd_vel.linear.x) > 1.0):
            self.last_cmd_vel = msg
            left_vel = Float64()
            right_vel = Float64()

            left_vel.data = 0.0
            right_vel.data = 0.0

            if (msg.angular.z == 0):
                left_vel.data = msg.linear.x
                right_vel.data = -msg.linear.x
            elif (msg.angular.z == 50):
                left_vel.data = -65.0
                right_vel.data = -65.0
            elif (msg.angular.z == -50):
                left_vel.data = 65.0
                right_vel.data = 65.0

            self.left_tread_pub.publish(left_vel)
            self.right_tread_pub.publish(right_vel)

    def recieve_arm_state(self, msg: Int16):
        #send value if it is new
        if (msg.data != self.last_cmd_arm):
            self.last_cmd_arm = msg

            arm_state = Float64()
            arm_state.data = float(msg.data)

            self.arm_state_pub.publish(arm_state)

    def recieve_bucket_state(self, msg: Int16):
        #send value if it is new
        if (msg.data != self.last_cmd_bucket):
            self.last_cmd_bucket = msg

            bucket_state = Float64()
            bucket_state.data = float(msg.data)

            self.bucket_state_pub.publish(bucket_state)


def main(args=None):
    rclpy.init(args=args) #initialize ros2 communication
    node = BaseControl() #instantiate a node (starts node)
    rclpy.spin(node) # .spin runs a node until it is manually killed
    rclpy.shutdown() # end ros2 comms


if __name__ == '__main__':
    main()