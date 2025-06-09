from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # VIO Node
        Node(
            package='farmbot_autonomous_pkg',
            executable='visual_imu',
            name='visual_imu',
            output='screen'
        ),
        
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # TF Static Transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '0', '0.5', '0', '0', '0', 'base_link', 'camera_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),
        
        # EKF Node (optional)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            output='screen',
            parameters=[{
                'odom0': '/vio/odometry',
                'imu0': '/imu/data',
                'world_frame': 'odom',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'odom0_config': [True, True, True,   # X, Y, Z
                                False, False, False, # roll, pitch, yaw
                                False, False, False,
                                False, False, False,
                                False, False, False],
                'imu0_config': [False, False, False,
                               True, True, True,    # roll, pitch, yaw
                               True, True, True],   # angular velocities
                'frequency': 50.0
            }]
        )
    ])
