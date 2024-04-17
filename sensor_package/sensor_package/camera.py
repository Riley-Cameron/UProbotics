import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class MultiCameraPublisher(Node):
    def __init__(self):
        super().__init__('multi_camera_publisher')
        self.cv_bridge = CvBridge()

        # Create VideoCapture objects for each camera
        self.cap1 = cv2.VideoCapture(0)
        self.cap2 = cv2.VideoCapture(1)

        # Create publishers for each camera
        self.pub1 = self.create_publisher(Image, 'camera1/image_raw', 10)
        self.pub2 = self.create_publisher(Image, 'camera2/image_raw', 10)

        # Create timer to publish frames
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret1, frame1 = self.cap1.read()
        ret2, frame2 = self.cap2.read()

        if ret1:
            msg1 = self.cv_bridge.cv2_to_imgmsg(frame1, 'bgr8')
            self.pub1.publish(msg1)

        if ret2:
            msg2 = self.cv_bridge.cv2_to_imgmsg(frame2, 'bgr8')
            self.pub2.publish(msg2)

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
