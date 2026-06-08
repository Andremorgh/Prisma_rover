#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class FrameGrabber(Node):
    def __init__(self):
        super().__init__('frame_grabber')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/prisma_rover/color/image_raw',
            self.image_callback,
            10
        )
        self.received = False

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Save to the specific temp media path/artifact path
            cv2.imwrite('/home/user/ros2_ws/src/camera_frame.png', cv_img)
            self.get_logger().info('Saved frame to /home/user/ros2_ws/src/camera_frame.png')
            self.received = True
            # We exit cleanly
            sys.exit(0)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            sys.exit(1)

def main():
    rclpy.init()
    grabber = FrameGrabber()
    rclpy.spin(grabber)

if __name__ == '__main__':
    main()
