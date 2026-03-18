import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('stm32_bot')
    rviz_config = os.path.join(pkg, 'config', 'imu_debug.rviz')

    return LaunchDescription([

        # IMU
        Node(
            package='tm_imu',
            executable='transducer_m_imu',
            name='tm_imu',
            parameters=[{'imu_port': '/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3170365A3334-if00'}]
        ),

        # Convertit /imu_data.orientation → TF world→imu_visual (frame dynamique)
        Node(
            package='stm32_bot',
            executable='imu_visualizer',
            name='imu_visualizer',
            output='screen'
        ),

        # Frame de référence fixe : world → base_link (identité)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'world', '--child-frame-id', 'base_link'
            ]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
