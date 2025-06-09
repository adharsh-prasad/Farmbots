#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header

import numpy as np

# Fake GPS origin (can be real location of the field)
SIM_ORIGIN_LAT = 37.4275
SIM_ORIGIN_LON = -122.1697
LAT_SCALE = 1e-5  # ~1 meter per 1e-5 deg
LON_SCALE = 1e-5

class GPSPoseBridge(Node):
    def __init__(self):
        super().__init__('gps_pose_bridge')

        self.declare_parameter('robot_name', 'husky_robot')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        self.get_logger().info(f"GPS bridge node started for model: {self.robot_name}")

    def model_states_callback(self, msg):
        self.get_logger().info("hi")
        if self.robot_name not in msg.name:
            self.get_logger().warn_once(f"Model '{self.robot_name}' not found in /gazebo/model_states.")
            return

        
        idx = msg.name.index(self.robot_name)
        position = msg.pose[idx].position
        

        lat = SIM_ORIGIN_LAT + position.x * LAT_SCALE
        lon = SIM_ORIGIN_LON + position.y * LON_SCALE
        alt = position.z

        gps_msg = NavSatFix()
        gps_msg.header = Header()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_link"

        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS

        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt

        gps_msg.position_covariance = [0.01, 0.0, 0.0,
                                       0.0, 0.01, 0.0,
                                       0.0, 0.0, 0.05]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.gps_pub.publish(gps_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GPSPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
