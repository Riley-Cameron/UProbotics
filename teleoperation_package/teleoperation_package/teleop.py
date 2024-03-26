#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import pygame
import time


class TeleopNode(Node):
    STOPPED = 0
    EXTEND = 1
    RETRACT = 2

    def __init__(self):
        super().__init__("teleop_node")
        self.joystick = None
        self.cmd_vel = Twist()
        self.arm_state = Int16()
        self.bucket_state = Int16()

        # Create Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd/velocity", 10)
        self.cmd_arm_state_pub = self.create_publisher(Int16, "/cmd/arm_state", 10)
        self.cmd_bucket_state_pub = self.create_publisher(Int16, "/cmd/bucket_state", 10)

        # Initialize Pygame modules for joystick handling.
        pygame.init()
        pygame.joystick.init()

        # Check if at least one joystick is connected.
        if pygame.joystick.get_count() > 0:
            # Initialize the first joystick.
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"Joystick initialized: {self.joystick.get_name()}")
        else:
            # Exit if no joystick is found.
            print("No joystick detected.")
            pygame.quit()
            exit()

        self.loop()

    def loop(self):
        try:
            while True:
                # Ensure Pygame processes internal events.
                pygame.event.pump()

                #DRIVING:
                # Read the joystick values
                drive_axis = self.joystick.get_axis(1) #left stick
                turn_left = self.joystick.get_button(4) #left bumper
                turn_right = self.joystick.get_button(5) #right bumper
                
                # Convert the joystick Y-axis position to a speed value (0-255).
                self.cmd_vel.linear.x = float(drive_axis * -100)

                #set turning values if x vel is 0
                if abs(self.cmd_vel.linear.x) < 1.0:
                    if (turn_left and not turn_right):
                        self.cmd_vel.angular.z = -50.0
                    elif (turn_right and not turn_left):
                        self.cmd_vel.angular.z = 50.0
                    else:
                        self.cmd_vel.angular.z = 0.0
                else:
                    self.cmd_vel.angular.z = 0.0
                
                # Publish speed value
                self.cmd_vel_pub.publish(self.cmd_vel)
                

                #BUCKET ARM:
                # read joystick values
                arm_extend = self.joystick.get_button(3) #Y button
                arm_retract = self.joystick.get_button(0) #A button
                bucket_down = self.joystick.get_button(2) #X button
                # for event in pygame.event.get():
                #     if event.type == pygame.JOYBUTTONDOWN:
                #         print("Button pressed:", event.button)
                #     elif event.type == pygame.JOYAXISMOTION:
                #         print("Axis motion - Axis:", event.axis, "Value:", event.value)
                
                # Wait a bit before sending the next command to avoid overwhelming the server.
                time.sleep(1)
        except KeyboardInterrupt:
            # Handle user interrupt (Ctrl+C).
            print("Program terminated by user.")
            pygame.quit()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args) #initialize ros2 communication
    teleop_node = TeleopNode() #instantiate a node (starts node)
    rclpy.spin(teleop_node) # .spin runs a node until it is manually killed

    rclpy.shutdown() # end ros2 comms

if __name__ == '__main__':
    main()

    

