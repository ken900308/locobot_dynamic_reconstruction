from setuptools import setup
from glob import glob
import os

package_name = "stretch3_ros_nodes"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Stretch3 ROS2 nodes (IPC bridge, auto anchor, pc2 to map)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ipc_bridge_node = stretch3_ros_nodes.ipc_bridge_node:main",
            "ipc_pointcloud_receiver = stretch3_ros_nodes.ipc_pointcloud_receiver:main",
            "auto_anchor_from_pointcloud_stretch3 = stretch3_ros_nodes.auto_anchor_from_pointcloud_stretch3:main",
            "pc2_to_map_stretch3 = stretch3_ros_nodes.pc2_to_map_stretch3:main",
            "multi_robot_fusion_node = stretch3_ros_nodes.multi_robot_fusion_node:main",
        ],
    },
)
