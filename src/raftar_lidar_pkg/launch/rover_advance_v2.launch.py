"""
rover_advance_v2.launch.py
~~~~~~~~~~~~~~~~~~~~~~~~~~
The whole obstacle-avoidance stack in one command:

    ros2 launch raftar_lidar_pkg rover_advance_v2.launch.py

Set start_mavros:=false if you would rather run MAVROS in its own terminal:

    ros2 launch raftar_lidar_pkg rover_advance_v2.launch.py start_mavros:=false

Every physical number here (mount heights, angles, serial ports) is
imported from raftar_lidar_pkg/rover_config.py rather than typed in, so it
cannot drift out of step with what the nodes believe.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from raftar_lidar_pkg import rover_config as cfg


def _static_tf(name, xyz, yaw_deg, parent, child):
    """One fixed 'this part is bolted here' relationship in the TF tree."""
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

    nav2_params = os.path.join(
        get_package_share_directory('raftar_lidar_pkg'),
        'config', 'nav2_local_costmap.yaml')

    start_mavros = LaunchConfiguration('start_mavros')

    return LaunchDescription([

        DeclareLaunchArgument(
            'start_mavros', default_value='true',
            description='Start MAVROS here. Set false to run it yourself.'),

        # ------------------------------------------------------------------
        # Link to the flight controller
        # ------------------------------------------------------------------
        # apm.launch is XML, so AnyLaunchDescriptionSource is required -
        # PythonLaunchDescriptionSource cannot parse it.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                os.path.join(get_package_share_directory('mavros'), 'launch', 'apm.launch')
            ]),
            launch_arguments={'fcu_url': cfg.FCU_URL}.items(),
            condition=IfCondition(start_mavros),
        ),

        # ------------------------------------------------------------------
        # LiDAR drivers
        # ------------------------------------------------------------------
        # Both are remapped explicitly so the topic names match what the
        # costmap and the mode switcher expect.
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_top',
            output='screen',
            parameters=[{
                'serial_port': cfg.TOP_LIDAR_PORT,
                'serial_baudrate': cfg.TOP_LIDAR_BAUD,
                'frame_id': cfg.TOP_LIDAR_FRAME,
                'scan_mode': 'Standard',
            }],
            remappings=[('scan', cfg.TOP_SCAN_TOPIC)],
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_low',
            output='screen',
            parameters=[{
                'serial_port': cfg.LOW_LIDAR_PORT,
                'serial_baudrate': cfg.LOW_LIDAR_BAUD,
                'frame_id': cfg.LOW_LIDAR_FRAME,
                'scan_mode': 'Standard',
            }],
            remappings=[('scan', cfg.LOW_SCAN_TOPIC)],
        ),

        # ------------------------------------------------------------------
        # Where everything is bolted (the TF tree)
        # ------------------------------------------------------------------
        #   map -> odom          static, here
        #   odom -> base_link    live, from coordinator_node
        #   base_link -> lasers  static, here
        _static_tf('map_to_odom', (0.0, 0.0, 0.0), 0.0,
                   cfg.MAP_FRAME, cfg.ODOM_FRAME),
        _static_tf('base_link_to_laser_top', cfg.TOP_LIDAR_XYZ, cfg.TOP_LIDAR_YAW_DEG,
                   cfg.BASE_FRAME, cfg.TOP_LIDAR_FRAME),
        _static_tf('base_link_to_laser_low', cfg.LOW_LIDAR_XYZ, cfg.LOW_LIDAR_YAW_DEG,
                   cfg.BASE_FRAME, cfg.LOW_LIDAR_FRAME),

        # ------------------------------------------------------------------
        # Obstacle detection + flight mode
        # ------------------------------------------------------------------
        Node(
            package='raftar_lidar_pkg',
            executable='obstacle_mode_switcher_dual',
            name='obstacle_mode_switcher_dual',
            output='screen',
            parameters=[{
                'scan_topic_top': cfg.TOP_SCAN_TOPIC,
                'scan_topic_low': cfg.LOW_SCAN_TOPIC,
                'threshold_m_top': cfg.OBSTACLE_THRESHOLD_TOP_M,
                'threshold_m_low': cfg.OBSTACLE_THRESHOLD_LOW_M,
                # Derived from the same mount angles used by the transforms
                # above, so the detector and the costmap always agree on
                # which way each sensor faces.
                'top_center_deg': cfg.forward_bearing_deg(cfg.TOP_LIDAR_YAW_DEG),
                'low_center_deg': cfg.forward_bearing_deg(cfg.LOW_LIDAR_YAW_DEG),
                'top_half_deg': cfg.DETECTION_HALF_ANGLE_DEG,
                'low_half_deg': cfg.DETECTION_HALF_ANGLE_DEG,
                'min_valid_m': 0.05,
                'debounce_sec': 0.5,
                'cooldown_sec': 2.0,
                'mode_when_obstacle': 'GUIDED',
                'mode_when_clear': 'AUTO',
            }],
        ),

        # ------------------------------------------------------------------
        # Where to drive while dodging
        # ------------------------------------------------------------------
        Node(
            package='raftar_lidar_pkg',
            executable='coordinator_node',
            name='coordinator_node',
            output='screen',
            parameters=[{
                'stuck_timeout_sec': 8.0,
                'min_progress_m': 0.1,
                'cmd_vel_topic': cfg.CMD_VEL_TOPIC,
                # Set false if MAVROS is configured to publish odom->base_link.
                'publish_odom_tf': True,
            }],
        ),

        # ------------------------------------------------------------------
        # Nav2: costmap + controller, then the smoother
        # ------------------------------------------------------------------
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
        ),

        # Nav2 servers start up "unconfigured" and do nothing until something
        # walks them through their lifecycle.  This is that something.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{
                'autostart': True,
                'use_sim_time': False,
                'node_names': ['controller_server', 'velocity_smoother'],
            }],
        ),

        # ------------------------------------------------------------------
        # Velocity commands -> flight controller
        # ------------------------------------------------------------------
        Node(
            package='raftar_lidar_pkg',
            executable='mav_to_teleop',
            name='mav_to_teleop',
            output='screen',
            parameters=[{
                # Reads the smoothed stream, not the raw one.
                'input_topic': cfg.CMD_VEL_SMOOTHED_TOPIC,
                'output_topic': cfg.SETPOINT_TOPIC,
                'invert_yaw_rate': True,
                'publish_rate_hz': 20.0,
                'cmd_timeout_sec': 0.5,
            }],
        ),

    ])
