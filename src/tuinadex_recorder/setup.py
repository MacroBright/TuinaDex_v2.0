import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tuinadex_recorder'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MacroBright',
    maintainer_email='bright@todo.todo',
    description='TuinaDex rosbag2 Recording Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tuinadex_recorder_node = tuinadex_recorder.recorder_node:main',
        ],
    },
)
