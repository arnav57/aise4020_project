from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'custom_car'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line is critical if you want to use Launch Files later!
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arnav',
    maintainer_email='agoyal57@uwo.ca',
    description='AISE 4020 Project: Modular Yahboom Car Control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_node   = custom_car.nodes.drive_node:main',
            'lidar_node   = custom_car.nodes.lidar_node:main',
            'vision_node  = custom_car.nodes.vision_node:main',
            'pid_node     = custom_car.nodes.pid_node:main',
            'arbiter_node = custom_car.nodes.arbiter_node:main',
            'web_node     = custom_car.nodes.web_node:main',
        ],
    },
)