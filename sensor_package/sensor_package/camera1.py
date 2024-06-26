#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                        
from rclpy.node import Node         
from sensor_msgs.msg import Image   
from cv_bridge import CvBridge     
import cv2                          

class ImagePublisher(Node):

    def __init__(self, name):
        super().__init__(name)                                           
        self.pub1 = self.create_publisher(Image, 'image1_raw', 10)  
        self.timer = self.create_timer(0.2, self.timer_callback)         
        self.cap1 = cv2.VideoCapture(0)                                  
        self.cv_bridge = CvBridge()     

        self.get_logger().info("Initialized Camera Node")                                 

    def timer_callback(self):
        ret1, frame1 = self.cap1.read()                                     
        if ret1 == True:                                                  
            self.pub1.publish(
                self.cv_bridge.cv2_to_imgmsg(frame1, 'bgr8'))          

        #self.get_logger().info('Publishing video frame')                 

def main(args=None):                                 
    rclpy.init(args=args)                            
    node = ImagePublisher("topic_webcam_pub")        
    rclpy.spin(node)                                 
    node.destroy_node()                              
    rclpy.shutdown()                                 