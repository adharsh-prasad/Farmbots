#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from tf_transformations import quaternion_from_matrix

class VONode(Node):
    def __init__(self):
    	super().__init__('vo_node')
    
	    self.subscription = self.create_subscription(
        	Image, '/front_camera/image_raw', self.process_image, 10
    	)
    
    	self.odom_pub = self.create_publisher(Odometry, '/vo/odometry', 10)
    
    	self.odom_msg = Odometry()
    	self.odom_msg.header.frame_id = "odom"
    	self.odom_msg.child_frame_id = "camera_optical_frame"  # Match your camera frame
    
    	self.bridge = CvBridge()
    	self.prev_gray = None
    	self.prev_kp = None
    	self.prev_des = None
    
    	self.orb = cv2.ORB_create(nfeatures=2000)
    	self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    
    	fx = fy = 554.3827128226441
    	cx = 320.5
    	cy = 240.5
    	self.K = np.array([[fx, 0, cx],
                       	[0, fy, cy],
                       	[0, 0, 1]])
    
    	self.pose = np.identity(4)
    	self.pose[2,3] = 1.5  # Example: camera 1.5m above ground
    	self.get_logger().info("VO Node Initialized")


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
                        
                        # Convert rotation matrix to quaternion
            		q = quaternion_from_matrix(self.pose)
            
            		# Populate odometry message
            		self.odom_msg.header.stamp = self.get_clock().now().to_msg()
            		self.odom_msg.pose.pose.position.x = self.pose[0,3]
            		self.odom_msg.pose.pose.position.y = self.pose[1,3]
            		self.odom_msg.pose.pose.position.z = self.pose[2,3]
            		self.odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            
            		# Add realistic covariance (adjust values based on your VO uncertainty)
            		self.odom_msg.pose.covariance = [
                		0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # X covariance
                		0.0, 0.1, 0.0, 0.0, 0.0, 0.0,  # Y covariance
                		0.0, 0.0, 0.1, 0.0, 0.0, 0.0,  # Z covariance
                		0.0, 0.0, 0.0, 0.05, 0.0, 0.0,  # Roll
                		0.0, 0.0, 0.0, 0.0, 0.05, 0.0,  # Pitch
                		0.0, 0.0, 0.0, 0.0, 0.0, 0.05   # Yaw
            		]
            
	            	self.odom_pub.publish(self.odom_msg)


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

