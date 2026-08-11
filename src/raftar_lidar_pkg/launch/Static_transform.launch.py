from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Static transform: base_link -> laser_frame
        # x=1.0m forward, z=0.5m height, no rotation
         # ----------------------------------------------------------------
                # 1a. Static Transform: base_link -> laser_top (1.15m above ground)
                # ----------------------------------------------------------------
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='base_link_to_laser_top',
                    arguments=[
                        '--x', '0.73',        # centered front/rear on rover
                        '--y', '0.0',        # centered left/right
                        '--z', '1.15',       # top LiDAR height from ground
                        '--roll', '0.0',
                        '--pitch', '0.0',
                        '--yaw', '0.0',
                        '--frame-id', 'base_link',
                        '--child-frame-id', 'laser_top'
                    ],
                    output='screen'
                ),
        
                # ----------------------------------------------------------------
                # 1b. Static Transform: base_link -> laser_low (0.58m above ground)
                # ----------------------------------------------------------------
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='base_link_to_laser_low',
                    arguments=[
                        '--x', '1.08',        # centered front/rear on rover
                        '--y', '0.0',        # centered left/right
                        '--z', '0.58',       # low LiDAR height from ground
                        '--roll', '0.0',
                        '--pitch', '0.0',
                        '--yaw', '0.0',
                        '--frame-id', 'base_link',
                        '--child-frame-id', 'laser_low'
                    ],
                    output='screen'
                ),
        
                # ----------------------------------------------------------------
                # 1c. Static Transform: map -> odom (identity, both ENU at home)
                # ----------------------------------------------------------------
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='map_to_odom',
                    arguments=[
                        '--x', '0.0',
                        '--y', '0.0',
                        '--z', '0.0',
                        '--roll', '0.0',
                        '--pitch', '0.0',
                        '--yaw', '0.0',
                        '--frame-id', 'map',
                        '--child-frame-id', 'odom'
                    ],
                    output='screen'
                ),

    ])