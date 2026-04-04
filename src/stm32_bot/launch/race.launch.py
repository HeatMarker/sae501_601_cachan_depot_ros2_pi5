import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_stm32      = get_package_share_directory('stm32_bot')
    pkg_nav2       = get_package_share_directory('nav2_bringup')
    map_file       = os.path.expanduser('~/ros2_ws/src/stm32_bot/maps/maps.yaml')
    path_file      = os.path.expanduser('~/ros2_ws/src/stm32_bot/paths/track.yaml')

    record_mode = LaunchConfiguration('record_mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'record_mode',
            default_value='false',
            description='true = enregistrement chemin | false = course autonome'
        ),

        # ── Capteurs + EKF ──────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_stm32, 'launch', 'robot_bringup.launch.py')
            )
        ),

        # ── Localisation AMCL (carte sauvegardée) ───────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='link_to_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'map':          map_file,
                'use_sim_time': 'false',
            }.items()
        ),

        # ── Mode ENREGISTREMENT ──────────────────────────
        Node(
            package='stm32_bot',
            executable='path_recorder',
            name='path_recorder',
            output='screen',
            parameters=[{
                'output_file':      path_file,
                'min_dist':         0.15,
                'loop_close_dist':  0.5,
                'loop_min_points':  30,
            }],
            condition=IfCondition(record_mode),
        ),

        # ── Mode COURSE ──────────────────────────────────
        Node(
            package='stm32_bot',
            executable='race_controller',
            name='race_controller',
            output='screen',
            parameters=[{
                'path_file':            path_file,
                'lookahead_dist':       0.4,
                'max_speed':            1.5,   # vitesse max ligne droite
                'min_speed':            0.3,   # vitesse min virage serré
                'speed_curvature_gain': 1.0,   # ↑ = ralentit plus tôt dans les virages
                'obstacle_threshold':   1.5,
                'emergency_stop_dist':  0.25,
                'lateral_offset':       0.3,
                'overtake_speed_boost': 0.2,
            }],
            condition=UnlessCondition(record_mode),
        ),
    ])
