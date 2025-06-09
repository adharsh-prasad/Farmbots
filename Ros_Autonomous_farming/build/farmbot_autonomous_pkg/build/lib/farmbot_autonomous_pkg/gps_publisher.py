#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import NavSatFix, NavSatStatus
from pyproj import Proj, Transformer
import numpy as np

class gps_publisher(Node):
    def __init__(self):
        super().__init__('gnss_simulator')
        
        # Configuration
        self.origin_lat = 37.4275  # Set your base coordinates (e.g., test field)
        self.origin_lon = -122.1697
        self.utm_zone = '10S'  # Find your zone: https://mangomap.com/robertyoung/maps/69585/what-utm-zone-am-i-in-#
        
        # UTM Projection Setup
        self.utm_proj = Proj(proj='utm', zone=self.utm_zone, ellps='WGS84')
        self.origin_easting, self.origin_northing = self.utm_proj(
            self.origin_lon, self.origin_lat
        )
        
        # ROS Interfaces
        self.pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', 
                               self.model_callback, 10)
        
        self.get_logger().info(f"GNSS Simulator initialized for UTM zone {self.utm_zone}")

    def model_callback(self, msg):
        try:
            idx = msg.name.index('husky_robot')  # Your robot's model name
        except ValueError:
            return

        # Get Gazebo ENU coordinates (X=East, Y=North)
        x = msg.pose[idx].position.x  # Easting
        y = msg.pose[idx].position.y  # Northing
        z = msg.pose[idx].position.z
        
        # Convert to geographic coordinates
        easting = self.origin_easting + x
        northing = self.origin_northing + y
        lon, lat = self.utm_proj(easting, northing, inverse=True)
        
        # Simulate GNSS characteristics
        lat += np.random.normal(0, 0.0000045)  # ~0.5m horizontal error
        lon += np.random.normal(0, 0.0000045)
        alt = 50.0 + z + np.random.normal(0, 1.5)  # MSL altitude + noise
        
        # Create NavSatFix message
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_antenna"
        gps_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
        
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt
        
        # Realistic covariance matrix (5m accuracy)
        gps_msg.position_covariance = [
            25.0, 0.0, 0.0,
            0.0, 25.0, 0.0,
            0.0, 0.0, 100.0
        ]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        
        self.pub.publish(gps_msg)
        self.get_logger().debug(f"Published simulated GNSS: {lat:.6f}, {lon:.6f}")

def main(args=None):
    rclpy.init(args=args)
    node = gps_publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

