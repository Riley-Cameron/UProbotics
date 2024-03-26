import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class TeleopJoy(Node):
    def __init__(self):
        # Initialize the Node with the name 'teleop_joy'
        super().__init__('teleop_joy')
        
        # Subscribe to the '/joy' topic to receive joystick messages
        # The 'joy_callback' function is called when a new message is received
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Create publishers for sending commands to the left and right motor controllers
        self.left_motor_pub = self.create_publisher(Float64, '/motor/tread_left', 10)
        self.right_motor_pub = self.create_publisher(Float64, '/motor/tread_right', 10)
        
        # Create a publisher for sending commands to the actuator controller
        self.actuator_pub = self.create_publisher(Float64, '/actuator/control', 10)

    def joy_callback(self, msg):
        # This method handles joystick messages (input)
        
        # Get forward/reverse speed from the vertical axis of the left joystick
        forward_speed = msg.axes[1]
        
        # Get turning speed from the horizontal axis of the left joystick
        turn = msg.axes[0]

        # Calculate the motor speeds based on joystick positions
        left_speed = self.calculate_motor_speed(forward_speed, turn, direction="left")
        right_speed = self.calculate_motor_speed(forward_speed, turn, direction="right")

        # Publish the calculated speeds to the motor controllers
        self.left_motor_pub.publish(Float64(data=left_speed))
        self.right_motor_pub.publish(Float64(data=right_speed))

        # Handle the 'Y' and 'A' button presses for actuator control
        self.handle_actuator_buttons(msg.buttons)

    def calculate_motor_speed(self, forward_speed, turn, direction="left"):
        # Calculate motor speed taking into account direction for differential drive
        # 'direction' specifies which motor (left or right) we're calculating for
        if direction == "left":
            speed = forward_speed + turn
        else:  # right motor
            speed = forward_speed - turn
        
        # Scale the speed to a specific range and ensure it does not exceed limits
        # Here, we scale it to -100 to 100 for simplicity; adjust as needed for your hardware
        speed = max(min(speed * 100, 100), -100)
        
        return speed

    def handle_actuator_buttons(self, buttons):
        # This method sends commands to the actuator based on button presses
        
        # Initialize the actuator command
        actuator_command = Float64()
        
        # 'Y' button to extend the actuator
        if buttons[3] == 1:  # 'Y' button index, may need adjustment
            actuator_command.data = 1
            
        # 'A' button to retract the actuator
        elif buttons[0] == 1:  # 'A' button index, may need adjustment
            actuator_command.data = -1
            
        # No button pressed or other button is pressed, stop the actuator
        else:
            actuator_command.data = 0
            
        # Publish the actuator command
        self.actuator_pub.publish(actuator_command)

# Entry point for the program
def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of TeleopJoy and spin (wait for callbacks)
    teleop_joy = TeleopJoy()
    rclpy.spin(teleop_joy)
    
    # Cleanup and shutdown after spinning
    teleop_joy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




