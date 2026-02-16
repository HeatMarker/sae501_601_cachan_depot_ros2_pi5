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
        
        # --- AJOUTS IMPORTANTS ---
        # 1. Installe tous les fichiers de lancement (.launch.py)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. Installe tous les fichiers de configuration (.yaml)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spinetti',
    maintainer_email='spinetti@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_bridge = stm32_bot.stm32_bridge:main',
            'stm32_teleop = stm32_bot.stm32_teleop:main',
        ],
    },
)