#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import NavSatFix, NavSatStatus
from pyproj import Proj
import numpy as np

class gps_publisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')

        # Configuration - SET THESE VALUES FIRST!
        self.origin_lat = 37.4275  # Example: Stanford University
        self.origin_lon = -122.1697
        self.utm_zone_num = 10     # Zone number for your location
        self.hemisphere = 'north'  # 'north' or 'south'

        # UTM Projection Setup
        self.utm_proj = Proj(
            proj='utm',
            zone=self.utm_zone_num,
            ellps='WGS84',
            south=(self.hemisphere.lower() == 'south')
        )
        self.origin_easting, self.origin_northing = self.utm_proj(
            self.origin_lon, self.origin_lat
        )

        # ROS Interfaces
        self.publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

    def model_states_callback(self, msg):
        try:
            idx = msg.name.index('husky_robot')  # Your robot's model name
        except ValueError:
            return

        # Get Gazebo position (ENU frame)
        x = msg.pose[idx].position.x  # Easting
        y = msg.pose[idx].position.y  # Northing
        
        # Convert to UTM coordinates
        easting = self.origin_easting + x
        northing = self.origin_northing + y
        
        # Convert back to geographic coordinates
        lon, lat = self.utm_proj(easting, northing, inverse=True)
        
        # Add realistic noise
        lat += np.random.normal(0, 0.0000045)  # ~0.5m error
        lon += np.random.normal(0, 0.0000045)
        alt = 50.0 + np.random.normal(0, 1.0)  # MSL altitude
        
        # Create and publish NavSatFix message
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_link"
        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt
        
        self.publisher.publish(gps_msg)

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

