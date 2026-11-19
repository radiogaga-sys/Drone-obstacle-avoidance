#!/usr/bin/python3
"""
Standalone AprilTag detection pipeline for the downward USB camera.

This is the SAME pipeline as the one embedded in real_drone.launch.py —
use it to bench-test camera + detector + tag_pose_relay without the flight
stack (SLAM / planner / MAVROS).

Chain (no image_proc — see below):
  usb_cam_down  --/camera_down/image_raw/compressed (hardware MJPEG)-->  apriltag_detector
                --/camera_down/camera_info-------------------------->
  apriltag_detector  --TF camera_down -> tag36h11:<id>-->  tag_pose_relay
  tag_pose_relay     --/tag_detection/pose, /tag_detection/id-->

Why there is no image_proc:
  config/apriltag_config.yaml sets image_transport: compressed, so
  apriltag_ros subscribes to <image_rect>/compressed and connects straight to
  the hardware MJPEG stream published by usb_cam (pixel_format: raw_mjpeg).
  It consumes the ORIGINAL K+D from config/camera_down.yaml and OpenCV
  solvePnP handles undistortion internally. Inserting image_proc here would
  publish an /image_rect topic that nothing subscribes to, and its output
  would never reach the detector.

Usage:
  ros2 launch scouting_bridge apriltag_pipeline.launch.py
  ros2 launch scouting_bridge apriltag_pipeline.launch.py video_device:=/dev/video2

WARNING: image_width / image_height default to the calibrated 1280x720 mode.
  config/camera_down.yaml holds the intrinsics for that exact mode only.
  Changing the resolution does NOT rescale the intrinsics — tag poses will be
  wrong until you re-calibrate. Only override these for detection smoke tests.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Arguments (defaults mirror config/usb_cam_down.yaml) ---
    video_device_arg = DeclareLaunchArgument(
        "video_device", default_value="/dev/video0",
        description="USB camera device path (main chain uses /dev/video0)")
    image_width_arg = DeclareLaunchArgument(
        "image_width", default_value="1280",
        description="Camera image width (must match camera_down.yaml intrinsics)")
    image_height_arg = DeclareLaunchArgument(
        "image_height", default_value="720",
        description="Camera image height (must match camera_down.yaml intrinsics)")
    framerate_arg = DeclareLaunchArgument(
        "framerate", default_value="30.0",
        description="Camera framerate")

    video_device = LaunchConfiguration("video_device")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    framerate = LaunchConfiguration("framerate")

    # --- Paths ---
    pkg_scouting = get_package_share_directory("scouting_bridge")
    apriltag_config = os.path.join(pkg_scouting, "config", "apriltag_config.yaml")
    usb_cam_config = os.path.join(pkg_scouting, "config", "usb_cam_down.yaml")
    bridge_config = os.path.join(pkg_scouting, "config", "real_drone.yaml")

    # --- 1. USB camera driver ---
    # pixel_format: raw_mjpeg -> publishes /camera_down/image_raw/compressed
    # (hardware MJPEG passthrough) + /camera_down/camera_info
    usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_down",
        namespace="camera_down",
        output="screen",
        parameters=[usb_cam_config, {
            "video_device": video_device,
            "image_width": image_width,
            "image_height": image_height,
            "framerate": framerate,
        }],
    )

    # --- 2. AprilTag detector (identical remappings to the main chain) ---
    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_detector",
        output="screen",
        parameters=[apriltag_config],
        remappings=[
            ("image_rect", "/camera_down/image_raw"),
            ("camera_info", "/camera_down/camera_info"),
        ],
    )

    # --- 3. Tag pose relay (TF -> world PoseStamped) ---
    tag_pose_relay_node = Node(
        package="scouting_bridge",
        executable="tag_pose_relay",
        name="tag_pose_relay",
        output="screen",
        parameters=[bridge_config],
    )

    return LaunchDescription([
        video_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        usb_cam_node,
        apriltag_node,
        tag_pose_relay_node,
    ])
