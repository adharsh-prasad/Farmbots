#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, TwistWithCovariance
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class VIONode(Node):
    def __init__(self):
        super().__init__('vio_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Setup synchronized subscribers
        image_sub = Subscriber(self, Image, '/camera/image_raw')
        imu_sub = Subscriber(self, Imu, '/imu/data')
        
        # Approximate time synchronizer (0.1 sec slop)
        self.ts = ApproximateTimeSynchronizer([image_sub, imu_sub], 
                                            queue_size=10, 
                                            slop=0.1)
        self.ts.registerCallback(self.sync_callback)
        
        # Initialize ORB feature detector
        self.orb = cv2.ORB_create(nfeatures=2000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Camera parameters (adjust for your camera)
        self.fx = self.fy = 554.38
        self.cx = 320.5
        self.cy = 240.5
        self.K = np.array([[self.fx, 0, self.cx],
                         [0, self.fy, self.cy],
                         [0, 0, 1]])
        
        # State variables
        self.prev_frame = None
        self.prev_kp = None
        self.prev_des = None
        self.pose = np.eye(4)  # Current pose matrix
        self.velocity = np.zeros(3)  # Current velocity
        
        # IMU variables
        self.last_imu_time = None
        self.angular_velocity = np.zeros(3)
        self.linear_acceleration = np.zeros(3)
        
        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/vio/odometry', 10)
        
        self.get_logger().info("VIO Node Ready")

    def sync_callback(self, img_msg, imu_msg):
        # Process IMU data first
        self.process_imu(imu_msg)
        
        # Process visual odometry
        self.process_image(img_msg)
        
        # Publish fused odometry
        self.publish_odometry(img_msg.header.stamp)

    def process_imu(self, imu_msg):
        # Update angular velocity and linear acceleration
        self.angular_velocity = np.array([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        ])
        
        self.linear_acceleration = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])
        
        # IMU-based orientation estimation (roll/pitch)
        dt = 0.0
        if self.last_imu_time:
            dt = (self.get_clock().now().nanoseconds - 
                 self.last_imu_time.nanoseconds) * 1e-9
        self.last_imu_time = self.get_clock().now()

    def process_image(self, img_msg):
        # Convert image to grayscale
        frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect ORB features
        kp, des = self.orb.detectAndCompute(gray, None)
        
        if self.prev_frame is not None and self.prev_des is not None:
            # Match features between frames
            matches = self.bf.match(self.prev_des, des)
            matches = sorted(matches, key=lambda x: x.distance)
            
            if len(matches) > 20:
                # Get matched points
                pts1 = np.float32([self.prev_kp[m.queryIdx].pt for m in matches])
                pts2 = np.float32([kp[m.trainIdx].pt for m in matches])
                
                # Compute essential matrix
                E, mask = cv2.findEssentialMat(pts1, pts2, self.K, 
                                              method=cv2.RANSAC, 
                                              prob=0.999, 
                                              threshold=1.0)
                
                if E is not None:
                    # Recover relative pose
                    _, R, t, _ = cv2.recoverPose(E, pts1, pts2, self.K, mask=mask)
                    
                    # IMU-assisted scale estimation
                    scale = self.estimate_scale(t)
                    
                    # Update pose
                    T = np.eye(4)
                    T[:3, :3] = R
                    T[:3, 3] = t.flatten() * scale
                    self.pose = self.pose @ T
                    
                    # IMU-assisted orientation correction
                    self.apply_imu_correction(R)

        # Update previous frame data
        self.prev_frame = gray
        self.prev_kp = kp
        self.prev_des = des

    def estimate_scale(self, t):
        """Use IMU acceleration to estimate scale"""
        accel_norm = np.linalg.norm(self.linear_acceleration)
        if accel_norm < 1e-6:
            return 0.0
            
        # Assume 30Hz camera frame rate
        dt = 1.0/30.0
        expected_motion = 0.5 * accel_norm * dt**2
        current_motion = np.linalg.norm(t)
        
        if current_motion < 1e-6:
            return 0.0
            
        return expected_motion / current_motion

    def apply_imu_correction(self, R):
        """Fuse IMU orientation with visual rotation"""
        # Convert IMU quaternion to rotation matrix
        roll, pitch, _ = euler_from_quaternion([
            self.angular_velocity[0],
            self.angular_velocity[1],
            self.angular_velocity[2],
            0.0
        ])
        
        # Create rotation matrix from IMU angles
        R_imu, _ = cv2.Rodrigues(np.array([roll, pitch, 0.0]))
        
        # Fuse rotations
        self.pose[:3, :3] = R_imu @ self.pose[:3, :3]

    def publish_odometry(self, stamp):
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link_vo"
        
        # Set position
        odom_msg.pose.pose.position.x = self.pose[0, 3]
        odom_msg.pose.pose.position.y = self.pose[1, 3]
        odom_msg.pose.pose.position.z = self.pose[2, 3]
        
        # Convert rotation matrix to quaternion
        yaw = np.arctan2(self.pose[1, 0], self.pose[0, 0])
        q = quaternion_from_euler(0.0, 0.0, yaw)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # Set covariance (tune these values)
        odom_msg.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.3, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VIONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
