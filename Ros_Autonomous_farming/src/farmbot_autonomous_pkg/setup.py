from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'farmbot_autonomous_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),glob('launch/*')),
        (os.path.join('share', package_name, 'worlds'),glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asgard',
    maintainer_email='asgard@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = farmbot_autonomous_pkg.vision_node:main',
            'vis_odem_node = farmbot_autonomous_pkg.vis_odem_node:main',
            'gps_publisher = farmbot_autonomous_pkg.gps_publisher:main',
            'yolo_node = farmbot_autonomous_pkg.yolo_node:main',
            'gps_pose_bridge = farmbot_autonomous_pkg.gps_pose_bridge:main',
            'visual_imu = farmbot_autonomous_pkg.visual_imu:main',
        ],
    },
)
