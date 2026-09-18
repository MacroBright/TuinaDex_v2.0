import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tuinadex_hand'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MacroBright',
    maintainer_email='bright@todo.todo',
    description='TuinaDex 16-DOF LEAP Hand ROS2 Runtime Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leap_hand_node = tuinadex_hand.leap_hand_node:main',
        ],
    },
)
