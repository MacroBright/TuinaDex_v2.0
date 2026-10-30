import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tuinadex_sim'


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


extra_mesh_files = package_files('meshes', os.path.join('share', package_name, 'meshes'))

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.xml')),
    ] + extra_mesh_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MacroBright',
    maintainer_email='bright@todo.todo',
    description='TuinaDex MuJoCo Digital Twin Simulation Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leap_hand_sim_node = tuinadex_sim.leap_hand_sim_node:main',
        ],
    },
)
