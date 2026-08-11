import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    nav2_params = os.path.join(
        get_package_share_directory('raftar_lidar_pkg'),
        'config',
        'nav2_local_costmap.yaml'
    )

    return LaunchDescription([

        # ============================================================
        # TERMINAL 1: MAVROS (ArduPilot CubeOrange+)
        # ============================================================
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                os.path.join(get_package_share_directory('mavros'), 'launch', 'apm.launch')
            ]),
            launch_arguments={
                'fcu_url': 'serial:///dev/serial/by-id/usb-CubePilot_CubeOrange+_230042001851323031393637-if00:115200',
            }.items(),
        ),

        # ============================================================
        # TERMINAL 2: RPLIDAR 1 (top LiDAR, baud 256000)
        # ============================================================
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar1',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_393144b960165644bc33f96580944cbe-if00-port0',
                'serial_baudrate': 256000,
                'frame_id': 'laser_frame',
                'scan_mode': 'Standard',
            }],
        ),

        # ============================================================
        # TERMINAL 3: RPLIDAR 2 (low LiDAR, remapped to /scan_low)
        # ============================================================
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar2',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_low',
                'scan_mode': 'Standard',
            }],
            remappings=[
                ('scan', '/scan_low'),
            ],
        ),

        # ============================================================
        # STATIC TRANSFORMS
        # ============================================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_top',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '1.15',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser_frame'
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_low',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.58',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser_low'
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom'
            ],
            output='screen'
        ),

        # ============================================================
        # DUAL-LiDAR OBSTACLE MODE SWITCHER (MANUAL lock)
        # ============================================================
        Node(
            package='raftar_lidar_pkg',
            executable='obstacle_mode_switcher_dual',
            name='obstacle_mode_switcher_dual',
            output='screen',
            parameters=[{
                'scan_topic_top':       '/scan',          # rplidar1 publishes here
                'scan_topic_low':       '/scan_low',      # rplidar2 remapped here
                'threshold_m_top':      2.0,
                'threshold_m_low':      2.0,
                'top_center_deg':       180.0,
                'top_half_deg':         45.0,
                'low_center_deg':       0.0,
                'low_half_deg':         45.0,
                'min_valid_m':          0.05,
                'debounce_sec':         0.5,
                'cooldown_sec':         2.0,
                'mode_when_obstacle':   'GUIDED',
                'mode_when_clear':      'AUTO',
            }]
        ),

        # ============================================================
        # COORDINATOR NODE
        # ============================================================
        Node(
            package='raftar_lidar_pkg',
            executable='coordinator_node',
            name='coordinator_node',
            output='screen',
            parameters=[{
                'mode_guided':  'GUIDED',
                'mode_auto':    'AUTO',
                'cooldown_sec': 2.0,
            }]
        ),

        # ============================================================
        # TERMINAL 5: mav_to_teleop
        # ============================================================
        Node(
            package='raftar_lidar_pkg',
            executable='mav_to_teleop',
            name='mav_to_teleop',
            output='screen',
        ),

        # ============================================================
        # NAV2 CONTROLLER SERVER (DWB)  +  LIFECYCLE MANAGER
        # ============================================================
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{
                'autostart':  True,
                'node_names': ['controller_server'],
            }]
        ),

    ])
