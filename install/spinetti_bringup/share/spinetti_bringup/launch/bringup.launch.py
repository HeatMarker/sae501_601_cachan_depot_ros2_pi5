import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Chemins vers les fichiers
    pkg_bringup = get_package_share_directory('spinetti_bringup')
    ekf_config_path = os.path.join(pkg_bringup, 'config', 'ekf.yaml')

    # --- INCLUSION DU LIDAR ---
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 
                         'launch', 'ld06.launch.py')
        )
    )

    return LaunchDescription([
        # 1. Le Bridge STM32 (Package stm32_bot)
        Node(
            package='stm32_bot',
            executable='stm32_bridge',
            name='stm32_bridge',
            output='screen'
        ),

        # 2. Le convertisseur de vitesse
        Node(
            package='spinetti_bringup',
            executable='odom_converter',
            name='odom_converter',
            output='screen'
        ),

        # 3. Le filtre EKF (robot_localization)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path], remappings=[('/odometry/filtered','/odom')]
        ),

        # 4. Inclusion du Launch File du LiDAR
        lidar_launch,

        # 4. Transformée statique : Position du LiDAR par rapport au centre de l'essieu arrière
        # Arguments : x y z yaw pitch roll parent_frame child_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=[
                '--x', '0.15',
                '--y', '0',
                '--z', '0.15',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'base_laser'
            ]
        ),
    ])