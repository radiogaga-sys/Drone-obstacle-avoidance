#!/usr/bin/python3
"""
FAST_LIO_ROS2 — real drone launch for Livox Mid-360.

Starts only the SLAM mapping node (no LiDAR driver — use the Livox
built-in launch for that). All parameters come from mid360.yaml.

Subscribes:
  - /livox/lidar  (livox_ros_driver2/CustomMsg)
  - /livox/imu    (sensor_msgs/Imu)

Publishes:
  - /Odometry               (camera_init -> body)
  - /cloud_registered       (camera_init)
  - /cloud_registered_body  (body)
  - /Laser_map, /path

Usage:
  ros2 launch fast_lio mid360_real.launch.py
  ros2 launch fast_lio mid360_real.launch.py use_rviz:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory("fast_lio")
    config_file = os.path.join(package_path, "config", "mid360.yaml")
    rviz_config = os.path.join(package_path, "rviz", "fastlio.rviz")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="false",
        description="Launch RViz2 (off by default on real drone)",
    )

    use_rviz = LaunchConfiguration("use_rviz")

    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="laser_mapping",
        output="screen",
        parameters=[config_file],
        # respawn deliberately OFF (was True + respawn_delay=2.0): FAST_LIO
        # fixes its world origin (camera_init) at process start, so a mid-flight
        # restart would move the origin to the drone's current position --
        # /odom_world jumps to ~0, ego-planner replans from a bogus pose, and
        # vision_pose_relay forwards that jump into PX4 EKF2. launch's respawn
        # ignores the exit code, so any crash triggered it. With respawn off a
        # crash just hovers in place (traj_server "hover when finish traj_").
        respawn=False,
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        use_rviz_arg,
        fast_lio_node,
        rviz_node,
    ])
