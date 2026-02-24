import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Chemins vers les paquets
    pkg_stm32 = FindPackageShare('stm32_bot').find('stm32_bot')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup').find('nav2_bringup')

    # Chemin exact vers le fichier YAML de ta carte
    map_file = os.path.join(pkg_stm32, 'maps', 'maps.yaml')

    return LaunchDescription([
        # RUSTINE : On crée le repère "base_footprint" réclamé par Nav2 et on le colle à "base_link"
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='link_to_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),

        # On appelle le launch file officiel de Nav2 pour la localisation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'false'
            }.items()
        )
    ])
