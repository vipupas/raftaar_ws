import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rp = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port':'/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            'frame_id': 'laser_frame',
            'scan_mode': 'Standard',
            'serial_baudrate': 115200
        }]
    )

    mavros = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[{
            'fcu_url': 'serial:///dev/serial/by-id/usb-CubePilot_CubeOrange+_230042001851323031393637-if00:57600'
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('raftar_lidar_pkg'),
            'rviz',
            'lidar.rviz'
        )]
    )

    lidar_monitor = Node(
        package='raftar_lidar_pkg',
        executable='mode_switch_obj_detect',
        name='lidar_monitor',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'threshold_m': 1.0, # distance in which object detect
            'front_angle_deg': 20.0, # angle in which object need to present 
            'debounce_sec': 0.5, # Debounce : obj must stay for this long
            'cooldown_sec': 2.0 # Cooldown : min time b/w mode change 
        }]
    )
    
    return LaunchDescription([
        mavros,
        rp,
        rviz,
        lidar_monitor
    ])                                                                                                                                                                          