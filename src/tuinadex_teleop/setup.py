import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tuinadex_teleop'


def package_files(directory, target_base):
    paths = []
    for path, directories, filenames in os.walk(directory):
        if not filenames:
            continue
        rel_dir = os.path.relpath(path, directory)
        dest = os.path.join(target_base, rel_dir) if rel_dir != '.' else target_base
        file_paths = [os.path.join(path, f) for f in filenames]
        paths.append((dest, file_paths))
    return paths


extra_data_files = package_files('configs', os.path.join('share', package_name, 'configs'))

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test', 'tests*']),
    package_data={
        package_name + '.tracking': ['models/*.task'],
        package_name + '.perception': ['models/*.task'],
        package_name + '.hub': ['static/*'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ] + extra_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MacroBright',
    maintainer_email='bright@todo.todo',
    description='TuinaDex Vision Teleoperation Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_gesture_node = tuinadex_teleop.teleop_gesture_node:main',
            'teleop_sim_composite_node = tuinadex_teleop.teleop_sim_composite_node:main',
        ],
    },
)
