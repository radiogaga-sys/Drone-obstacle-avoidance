from setuptools import find_packages, setup
import os
from glob import glob

package_name = "scouting_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Scouting Team",
    maintainer_email="user@example.com",
    description="Glue scripts bridging SLAM, PX4/MAVROS, and ego-planner for real drone deployment",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vision_pose_relay = scouting_bridge.vision_pose_relay:main",
            "slam_odom_relay = scouting_bridge.slam_odom_relay:main",
            "tf_broadcaster = scouting_bridge.tf_broadcaster:main",
            "tag_pose_relay = scouting_bridge.tag_pose_relay:main",
            "tag_mission_controller = scouting_bridge.tag_mission_controller:main",
        ],
    },
)
