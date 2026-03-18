import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. Inclusion LIDAR
    lidar_pkg_dir = get_package_share_directory('ldlidar_stl_ros2')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_pkg_dir, 'launch', 'ld06.launch.py'))
    )

    # 2. Config EKF (Attention au nom du fichier ici !)
    pkg_stm32 = get_package_share_directory('stm32_bot')
    # Assure-toi que ton fichier s'appelle bien ekf_config.yaml ou ekf.yaml
    ekf_config = os.path.join(pkg_stm32, 'config', 'ekf_config.yaml')

    return LaunchDescription([
        lidar_launch,

        # --- STM32 BRIDGE ---
        Node(
            package='stm32_bot',
            executable='stm32_bridge',
            name='stm32_bridge',
            parameters=[
                {'port': '/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF505075894967183959-if02'},
                {'debug_pid': False} # Mets True ici quand tu veux utiliser ton PC pour régler
            ]
        ),

        # --- IMU ---
        Node(
            package='tm_imu',
            executable='transducer_m_imu',
            name='tm_imu',
            # ID du Virtual ComPort (IMU)
            parameters=[{'imu_port': '/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3170365A3334-if00'}]
        ),

        # --- EKF (FUSION) ---
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),

        # --- STATIC TRANSFORMS (GEOMETRIE) ---
        
        # IMU DDD TM210 : X(rouge)=haut(+robot_Z), Y(vert)=gauche(+robot_Y), Z(bleu)=arrière(-robot_X)
        # R = Ry(-90°)  =>  pitch=-1.57, roll=0, yaw=0
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.08',
                '--yaw', '0', '--pitch', '-1.57', '--roll', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'imu'
            ]
        ),

        # LIDAR: 28.5cm devant (essieu arrière), 13.5cm de haut (sol)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.285', '--y', '0', '--z', '0.135',
                '--yaw', '-1.57', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'base_laser'
            ]
        ),
    ])