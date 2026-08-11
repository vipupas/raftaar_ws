import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'raftar_lidar_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vip',
    maintainer_email='s.pvimalpratap9458@gmail.com',
    description='this package is for all lidar related works',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_publisher = raftar_lidar_pkg.lidar_publisher:main',
            'mode_switch_obj_detect = raftar_lidar_pkg.object_detect_rplidar:main',
            'obstacle_mode_switcher = raftar_lidar_pkg.obstacle_mode_switcher_3:main',
            'mav_to_teleop = raftar_lidar_pkg.mav_to_cmd_vel:main',
            'coordinator_node = raftar_lidar_pkg.coordinator_node:main',
            'obstacle_mode_switcher_adv = raftar_lidar_pkg.obstacle_mode_switcher_adv:main',
            'obstacle_mode_switcher_dual = raftar_lidar_pkg.obstacle_mode_switcher_dual:main'
        ],
    },
)
