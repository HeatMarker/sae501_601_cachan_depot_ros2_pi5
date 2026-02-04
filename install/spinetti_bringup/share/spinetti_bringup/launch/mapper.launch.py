import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Chemins vers les fichiers
    pkg_share = get_package_share_directory('spinetti_bringup')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'bringup.launch.py')
        )
    )
    
    params_file = os.path.join(pkg_share, 'config', 'mapper_params.yaml')

    slam_node = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    output='screen',
    parameters=[
        params_file,
        {'use_sim_time': False}
    ]
    )

    return LaunchDescription([
        bringup_launch,
        slam_node
    ])