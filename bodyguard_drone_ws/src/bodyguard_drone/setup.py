from setuptools import setup
from glob import glob
import os

package_name = 'bodyguard_drone'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # World files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        # Model files
        (os.path.join('share', package_name, 'models/drone'), glob('models/drone/*')),
        (os.path.join('share', package_name, 'models/person_with_ring'), glob('models/person_with_ring/*')),
        (os.path.join('share', package_name, 'models/coffee_shop'), glob('models/coffee_shop/*')),
        (os.path.join('share', package_name, 'models/obstacles'), glob('models/obstacles/*')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bodyguard Drone Team',
    maintainer_email='team@bodyguarddrone.com',
    description='Bodyguard Drone Simulation with Gazebo, ROS 2, and ML perception',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = bodyguard_drone.perception_node:main',
            'navigation_node = bodyguard_drone.navigation_node:main',
            'user_event_node = bodyguard_drone.user_event_node:main',
            'voice_cmd_node = bodyguard_drone.voice_cmd_node:main',
            'tts_node = bodyguard_drone.tts_node:main',
            'drone_control_node = bodyguard_drone.drone_control_node:main',
            'command_parser_node = bodyguard_drone.command_parser_node:main',
        ],
    },
)
