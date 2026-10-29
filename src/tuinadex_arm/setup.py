import os
from glob import glob
from setuptools import find_packages, setup

package_name = "tuinadex_arm"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
]

# 递归安装 description 资源 (URDF 与 Meshes)
for root, dirs, files in os.walk("description"):
    if files:
        install_dir = os.path.join("share", package_name, root)
        file_paths = [os.path.join(root, f) for f in files]
        data_files.append((install_dir, file_paths))

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,

    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MacroBright",
    maintainer_email="bright@todo.todo",
    description="TuinaDex 6-DOF reBot B601-RS Robotic Arm ROS2 Runtime Package",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_node = tuinadex_arm.arm_node:main",
        ],
    },
)
