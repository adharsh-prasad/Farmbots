#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

class VONode(Node):
    def __init__(self):
        super().__init__('vo_node')

        self.subscription = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.process_image,
            10
        )

        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_kp = None
        self.prev_des = None

        self.orb = cv2.ORB_create(nfeatures=2000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Camera intrinsics (make sure these match your actual camera)
        fx = fy = 554.3827128226441
        cx = 320.5
        cy = 240.5
        self.K = np.array([[fx, 0, cx],
                           [0, fy, cy],
                           [0,  0,  1]])

        # Initialize pose
        self.pose = np.eye(4)
        self.get_logger().info("VO Node Initialized with pose estimation")

    def process_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        kp, des = self.orb.detectAndCompute(gray, None)

        if self.prev_gray is not None and self.prev_des is not None and des is not None:
            matches = self.bf.match(self.prev_des, des)
            matches = sorted(matches, key=lambda x: x.distance)

            if len(matches) > 10:
                pts1 = np.float32([self.prev_kp[m.queryIdx].pt for m in matches])
                pts2 = np.float32([kp[m.trainIdx].pt for m in matches])

                E, mask = cv2.findEssentialMat(pts1, pts2, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)

                if E is not None:
                    inliers1 = pts1[mask.ravel() == 1]
                    inliers2 = pts2[mask.ravel() == 1]

                    if len(inliers1) >= 5:  # avoid degenerate case
                        _, R, t, _ = cv2.recoverPose(E, inliers1, inliers2, self.K)

                        scale = np.linalg.norm(t)
                        step_size = 0.1
                        t_scaled = (t / scale) * step_size if scale > 1e-6 else t * step_size

                        T = np.eye(4)
                        T[:3, :3] = R
                        T[:3, 3] = t_scaled.squeeze()

                        self.pose = self.pose @ T
                        position = self.pose[:3, 3]

                        self.get_logger().info(f"Position: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")

        self.prev_gray = gray
        self.prev_kp = kp
        self.prev_des = des

    def destroy_node(self):
        super().destroy_node()
        self.get_logger().info("VO node shut down.")

def main(args=None):
    rclpy.init(args=args)
    node = VONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

