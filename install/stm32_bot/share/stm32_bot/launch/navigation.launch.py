import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_stm32_bot = get_package_share_directory('stm32_bot')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 1. Chemin vers ta CARTE
    map_file = os.path.join(pkg_stm32_bot, 'maps', 'my_track.yaml')

    # 2. Chemin vers ta CONFIG (Le fichier que tu viens de créer)
    params_file = os.path.join(pkg_stm32_bot, 'config', 'my_nav2_params.yaml')

    # 3. Lancer tes Drivers (Lidar, IMU, Roues, EKF)
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_stm32_bot, 'launch', 'robot_bringup.launch.py')
        )
    )

    # 4. Lancer Nav2 avec ta config perso
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': params_file, # <--- C'est ici que la magie opère !
        }.items()
    )

    # 5. Fix pour les pieds du robot (base_footprint)
    footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    return LaunchDescription([
        robot_bringup,
        nav2_bringup,
        footprint_tf
    ])