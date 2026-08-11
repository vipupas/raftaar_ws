"""
Static_transform.launch.py
~~~~~~~~~~~~~~~~~~~~~~~~~~
Just the fixed parts of the TF tree, on their own.  Useful for checking
sensor alignment in RViz without starting the whole stack.

    ros2 launch raftar_lidar_pkg Static_transform.launch.py

Do not run this alongside rover_advance*.launch.py - those publish the
same transforms, and two publishers of one transform makes the tree
jitter between them.

The frame names and offsets used to be written out by hand here and
disagreed with the ones in rover_advance*.launch.py (different child frame
names, different x offsets).  Both now read rover_config.py.
"""

import math

from launch import LaunchDescription
from launch_ros.actions import Node

from raftar_lidar_pkg import rover_config as cfg


def _static_tf(name, xyz, yaw_deg, parent, child):
    x, y, z = xyz
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        output='screen',
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--roll', '0.0', '--pitch', '0.0',
            '--yaw', str(math.radians(yaw_deg)),
            '--frame-id', parent,
            '--child-frame-id', child,
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        _static_tf('map_to_odom', (0.0, 0.0, 0.0), 0.0,
                   cfg.MAP_FRAME, cfg.ODOM_FRAME),
        _static_tf('base_link_to_laser_top', cfg.TOP_LIDAR_XYZ, cfg.TOP_LIDAR_YAW_DEG,
                   cfg.BASE_FRAME, cfg.TOP_LIDAR_FRAME),
        _static_tf('base_link_to_laser_low', cfg.LOW_LIDAR_XYZ, cfg.LOW_LIDAR_YAW_DEG,
                   cfg.BASE_FRAME, cfg.LOW_LIDAR_FRAME),
    ])
