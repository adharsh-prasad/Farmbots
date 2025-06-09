from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('farmbot_autonomous_pkg')

    world_path = os.path.join(pkg_share, 'worlds', 'farmland.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
            'pause': 'false',
            'use_sim_time': 'true'
        }.items()
    )

    gps_node = Node(
        package='farmbot_autonomous_pkg',
        executable='gps_pose_bridge',
        name='gps_pose_bridge',
        output='screen',
        parameters=[{
            'robot_name': 'husky_robot'
        }]
    )

    return LaunchDescription([
        gazebo,
        gps_node
    ])

