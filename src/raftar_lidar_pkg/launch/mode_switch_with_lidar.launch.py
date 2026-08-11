"""
mode_switch_with_lidar.launch.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The simple bench-test rig: one LiDAR, MAVROS, RViz, and the basic
"obstacle in front -> change mode" node.  No Nav2, no avoidance - it only
proves the sensing and the mode switching work.

    ros2 launch raftar_lidar_pkg mode_switch_with_lidar.launch.py

Note the mode this drops into when the path is clear is MANUAL, not AUTO,
so it hands control back to the RC transmitter rather than to a mission.
That is deliberate for bench testing.  rover_advance*.launch.py is the one
that resumes AUTO.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from raftar_lidar_pkg import rover_config as cfg


def generate_launch_description():

    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port': cfg.LOW_LIDAR_PORT,
            'serial_baudrate': cfg.LOW_LIDAR_BAUD,
            'frame_id': cfg.LOW_LIDAR_FRAME,
            'scan_mode': 'Standard',
        }],
    )

    mavros = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[{'fcu_url': cfg.FCU_URL}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('raftar_lidar_pkg'), 'rviz', 'lidar.rviz')],
    )

    lidar_monitor = Node(
        package='raftar_lidar_pkg',
        executable='mode_switch_obj_detect',
        name='lidar_monitor',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'threshold_m': 1.0,       # trigger when something is this close
            'front_angle_deg': 20.0,  # half-width of the cone watched
            'debounce_sec': 0.5,      # must persist this long to count
            'cooldown_sec': 2.0,      # minimum gap between mode changes
        }],
    )

    return LaunchDescription([mavros, lidar, rviz, lidar_monitor])
