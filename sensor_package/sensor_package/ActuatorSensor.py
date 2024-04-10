#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class ActuatorSensor(Node):

    STOPPED = 0.0
    EXTEND = 1.0
    RETRACT = 2.0

    """
    Virtual Encoder to track actuator position:
    Calculates a position as the percentage of a range. This is based on the given 'time to extend.'

    Parameters:
    signal_topic - topic to subscribe to for actuator state
    pos_topic - publishes the estimated position of the actuator as a percentage (0-100)
    node - node name
    extension_time - time in ms for actuator to extend to its full range

    """
    def __init__(self, signal_topic : str, pos_topic : str, node : str, extension_time : int):
        super().__init__(node)
        self.extension_time = extension_time
        self.position = 0.0 #track position in ms
        self.actuator_state = self.STOPPED #track actuator state

        self.sub = self.create_subscription(Float64, signal_topic, self.update_state, 10)
        self.position_pub = self.create_publisher(Float64, pos_topic, 10)

        self.loop()

    def update_state(self, msg: Float64):
        self.actuator_state = msg.data

    def loop(self):
        try:
            msg = Float64()
            cycles = 0
            while True:
                if (self.actuator_state == self.EXTEND):
                    self.position += 1.0
                elif (self.actuator_state == self.RETRACT):
                    self.position -= 1.0

                if (self.position < 0.0):
                    self.position = 0.0

                if (self.position > self.extension_time):
                    self.position = self.extension_time

                percentage = 100.0 * self.position / self.extension_time 
                if (cycles == 100):
                    msg.data = percentage
                    self.position_pub.publish(msg)
                    cycles = 0

                time.sleep(0.001)
                cycles += 1
                
        except KeyboardInterrupt:
            pass
