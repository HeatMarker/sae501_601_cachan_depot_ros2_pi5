import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stm32_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Installation des fichiers Launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Installation des fichiers de Configuration (YAML + RViz2)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
        # Installation des fichiers de Carte (Maps)
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spinetti',
    maintainer_email='spinetti@todo.todo',
    description='Package pour le contrôle du châssis TT-02 via STM32 et Manette PS3',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Commande = package.nom_du_fichier:fonction_main
            'stm32_bridge = stm32_bot.stm32_bridge:main',
            'stm32_teleop = stm32_bot.stm32_teleop:main',
            'ps3_mapper = stm32_bot.ps3_mapper:main',
            'imu_visualizer = stm32_bot.imu_visualizer:main',
        ],
    },
)