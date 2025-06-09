#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
from rclpy.qos import qos_profile_sensor_data
import os


class YoloDetector(Node):
    def __init__(self):        
        super().__init__('yolo_detector_node')
        self.output_dir = os.path.expanduser('~/farmbot/videos')
        os.makedirs(self.output_dir, exist_ok=True)
        self.frame_count = 0

        self.focal_length = 554.38  # from camera_info topic
        self.object_height_m = 1.0  # assumed known plant height in meters

        
        self.sub = self.create_subscription(Image, '/front_camera/image_raw', self.image_callback, qos_profile_sensor_data)
        self.annotated_pub = self.create_publisher(Image, '/yolo/image_annotated', 10)

        self.bridge = CvBridge()

        # Load the YOLOv8 model (replace with your preferred variant)
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info("YOLOv8 model loaded successfully.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return 

        results = self.model(frame)

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                label = self.model.names[cls_id]

                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                bbox_height = y2 - y1

                if bbox_height > 0:
                    estimated_distance = (self.object_height_m * self.focal_length) / bbox_height
                    self.get_logger().info(f"Estimated distance to {label}: {estimated_distance:.2f} meters")

                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{label} ({conf:.2f})", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Draw distance below bounding box
                    cv2.putText(frame, f"{estimated_distance:.2f}m", (x1, y2 + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                else:
                    self.get_logger().warn("Bounding box height is zero — cannot estimate distance.")

        # Publish the annotated frame
        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.annotated_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
