"""
rover_advance.launch.py
~~~~~~~~~~~~~~~~~~~~~~~
The full stack, but without starting MAVROS - for when you want MAVROS in
its own terminal so you can watch its output.

    ros2 launch raftar_lidar_pkg rover_advance.launch.py

This used to be a full copy of rover_advance_v2.launch.py with one line
commented out.  The two copies had already drifted apart, so this is now a
thin wrapper: there is exactly one description of the stack, in v2.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    v2 = os.path.join(
        get_package_share_directory('raftar_lidar_pkg'),
        'launch', 'rover_advance_v2.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(v2),
            launch_arguments={'start_mavros': 'false'}.items(),
        ),
    ])
