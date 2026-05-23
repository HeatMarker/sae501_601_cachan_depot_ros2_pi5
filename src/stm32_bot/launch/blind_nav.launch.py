import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('stm32_bot')

    max_speed   = LaunchConfiguration('max_speed')
    curve_gain  = LaunchConfiguration('curve_gain')

    return LaunchDescription([

        DeclareLaunchArgument(
            'max_speed',
            default_value='0.8',
            description='Vitesse max en ligne droite (m/s)'
        ),
        DeclareLaunchArgument(
            'curve_gain',
            default_value='2.0',
            description='Freinage en virage : 0=aucun, 2=modéré, 5=fort. '
                        'v = max_speed / (1 + gain × |steering|)'
        ),

        # ── Capteurs + EKF (bridge sans Ackermann — direction directe) ───
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'robot_bringup.launch.py')
            ),
            launch_arguments={'ackermann_mode': 'false'}.items()
        ),

        # ── Contrôleur blind nav ─────────────────────────────────────────
        Node(
            package='stm32_bot',
            executable='blind_nav',
            name='blind_nav',
            output='screen',
            parameters=[{
                # ── Args CLI ──────────────────────────────────────────────
                'max_speed':        ParameterValue(max_speed,  value_type=float),
                'speed_curve_gain': ParameterValue(curve_gain, value_type=float),

                # ── Vitesse (fixes) ───────────────────────────────────────
                'min_speed':            0.3,    # m/s — plancher vitesse en virage serré
                'dist_full_speed':      2.0,    # m — au-delà, vitesse max

                # ── Sécurité (voiture 18cm large, piste 60cm min) ─────────
                'emergency_stop_dist':  0.15,   # m — arrêt si obstacle droit devant
                'safety_bubble_radius': 0.15,   # m — masque autour obstacle proche

                # ── Espace libre ──────────────────────────────────────────
                'gap_threshold':        0.5,    # m — distance min pour compter comme "libre"
                'cone_half_deg':        70.0,   # ° — demi-cône de recherche

                # ── Direction ─────────────────────────────────────────────
                'steering_gain':        2.0,    # Kp — braquage max dès ~45° d'erreur
                'imu_d_gain':           0.0,    # Kd — activer (0.05) si oscillations

                # ── Arc arrêt urgence ─────────────────────────────────────
                'front_arc_deg':        20.0,   # ° de chaque côté du centre
            }]
        ),
    ])
