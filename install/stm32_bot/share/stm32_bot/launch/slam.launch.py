import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Chemin vers ton fichier YAML de paramètres
    pkg_stm32 = FindPackageShare('stm32_bot').find('stm32_bot')
    slam_params_file = os.path.join(pkg_stm32, 'config', 'mapper_params_online_async.yaml')

    # 2. Chemin vers le launch file officiel de slam_toolbox (qui gère l'activation)
    pkg_slam_toolbox = FindPackageShare('slam_toolbox').find('slam_toolbox')
    slam_launch_file = os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')

    return LaunchDescription([
        # Délai de 3s pour laisser l'EKF publier ses premiers TF avant que SLAM
        # ne tente son premier lookup — corrige le bug de la frame odom manquante
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(slam_launch_file),
                    launch_arguments={'slam_params_file': slam_params_file}.items()
                )
            ]
        )
    ])