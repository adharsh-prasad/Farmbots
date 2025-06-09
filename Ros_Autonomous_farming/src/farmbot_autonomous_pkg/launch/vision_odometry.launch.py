from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='visual_odometry',
            name='rtabmap_vo',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'approx_sync': True,
                'visual_odometry': True
            }],
            remappings=[
                ('image', '/front_camera/image_raw'),
                ('camera_info', '/front_camera/camera_info')
            ],
            output='screen'
        )
    ])
