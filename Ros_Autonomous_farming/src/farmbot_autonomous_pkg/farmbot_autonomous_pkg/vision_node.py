#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Create a subscriber to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.process_data,
            10
        )
        self.out = cv2.VideoWriter('/home/asgard/farmbot/src/computer_vision/vision.mp4', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30, (640, 480))
                
        self.get_logger().info("Vision Node Initialized")
        
        # Initialize the CvBridge
        self.bridge = CvBridge()
    
    def process_data(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.out.write(frame)
        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()