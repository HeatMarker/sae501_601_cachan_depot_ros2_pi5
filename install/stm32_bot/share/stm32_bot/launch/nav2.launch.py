import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_stm32 = FindPackageShare('stm32_bot').find('stm32_bot')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup').find('nav2_bringup')

    map_file = os.path.join(pkg_stm32, 'maps', 'maps.yaml')
    # Ajout de notre nouveau fichier de configuration personnalisé !
    nav2_params_file = os.path.join(pkg_stm32, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='link_to_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'false',
                'params_file': nav2_params_file   # On l'injecte ici
            }.items()
        )
    ])