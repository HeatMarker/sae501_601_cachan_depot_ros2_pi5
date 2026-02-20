from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Lecture de la manette PS3 (Port js1)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0
            }]
        ),

        # 2. Ton Mapper personnalisé (Logique : Croix + L2/R2 + Direction Inversée)
        # Ce nœud transforme les axes de la manette en message /cmd_vel
        Node(
            package='stm32_bot',
            executable='ps3_mapper',
            name='ps3_mapper',
            output='screen'
        ),
    ])
