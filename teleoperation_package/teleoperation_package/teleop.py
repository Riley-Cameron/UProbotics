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
            self.get_logger().info(f"Joystick initialized: {self.joystick.get_name()}")
        else:
            # Exit if no joystick is found.
            self.get_logger().info("No joystick detected.")
            pygame.quit()
            exit()

        # Define debounce delay
        self.debounce_delay = 0.2  # Adjust as needed (in seconds)
        self.last_button_states = [False] * self.joystick.get_numbuttons()
        self.last_button_times = [0] * self.joystick.get_numbuttons()

        self.get_logger().info("Teleoperation Node Initialized")

        self.loop()


    def debounce_button(self, button_index):
        current_state = self.joystick.get_button(button_index)
        current_time = pygame.time.get_ticks() / 1000.0  # Convert to seconds
        
        if current_state != self.last_button_states[button_index]:
            if current_time - self.last_button_times[button_index] >= self.debounce_delay:
                self.last_button_states[button_index] = current_state
                self.last_button_times[button_index] = current_time
                return current_state
        else:
            self.last_button_times[button_index] = current_time

        return False
    

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
                
                # Convert the joystick Y-axis position to a percentage value
                self.cmd_vel.linear.x = float(drive_axis * 100)

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
                

                #BUCKET & ARM:
                # read joystick values
                arm_up = self.joystick.get_button(3) #Y button
                arm_down = self.joystick.get_button(0) #A button
                bucket_down = self.joystick.get_button(2) #X button
                bucket_up = self.joystick.get_button(1) #B button

                #set arm state message
                if (arm_up and not arm_down):
                    self.arm_state.data = self.EXTEND
                elif (not arm_up and arm_down):
                    self.arm_state.data = self.RETRACT
                else:
                    self.arm_state.data = self.STOPPED

                #set bucket state message
                if (bucket_up and not bucket_down):
                    self.bucket_state.data = self.RETRACT
                elif (not bucket_up and bucket_down):
                    self.bucket_state.data = self.EXTEND
                else:
                    self.bucket_state.data = self.STOPPED

                #publish values
                self.cmd_bucket_state_pub.publish(self.bucket_state)
                self.cmd_arm_state_pub.publish(self.arm_state)

                
                # Wait a bit before sending the next command to avoid overwhelming the server.
                time.sleep(0.1)
        except KeyboardInterrupt:
            # Handle user interrupt (Ctrl+C).
            print("Program terminated by user.")
            pygame.quit()

def main(args=None):
    rclpy.init(args=args) #initialize ros2 communication
    teleop_node = TeleopNode() #instantiate a node (starts node)
    rclpy.spin(teleop_node) # .spin runs a node until it is manually killed

    rclpy.shutdown() # end ros2 comms

if __name__ == '__main__':
    main()

    

