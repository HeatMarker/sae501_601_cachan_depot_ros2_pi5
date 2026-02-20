from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Lecture de la manette
        Node(
            package='joy',
            executable='joy_node',
            parameters=[{'dev': '/dev/input/js1'}]
        ),
        # 2. Traduction Joy -> Cmd_vel
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'axis_linear.x': 1,
                'axis_angular.yaw': 0,
                'enable_button': 4, # L1 pour avancer
                'scale_linear.x': 3.0,
                'scale_angular.yaw': 1.0,
            }]
        ),
        # 3. Ton script Bridge vers la STM32
        Node(
            package='stm32_bridge', # À adapter selon ton nom de package
            executable='stm32_bridge.py',
            output='screen'
        )
    ])