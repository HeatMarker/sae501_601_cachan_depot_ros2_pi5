from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Déclaration de l'argument (3.0 m/s par défaut)
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='3.0',
        description='Vitesse max en m/s'
    )

    # 2. LE PILOTE MATÉRIEL (Il lit le Bluetooth/USB de la manette)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    # 3. TON SCRIPT (Il traduit les boutons en vitesses)
    teleop_node = Node(
        package='stm32_bot',
        executable='ps3_mapper',
        name='ps3_mapper',
        parameters=[{
            'max_speed': LaunchConfiguration('max_speed')
        }]
    )

    return LaunchDescription([
        max_speed_arg,
        joy_node,       # <- C'est lui qui manquait !
        teleop_node
    ])