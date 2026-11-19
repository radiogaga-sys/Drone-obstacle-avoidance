#!/usr/bin/env python3
"""
Launch FAST-LIVO2 SLAM for Mid-360 simulation.

Starts:
  1. IMU frame relay  — fixes Gazebo scoped frame_id -> mid360
  2. FAST-LIVO2 LIO   — LiDAR-inertial odometry + point clouds

Prerequisites:
  - PX4 SITL + Gazebo running (./run_px4_iris_mid360.sh)
  - LiDAR topic /mid360 + IMU topic /livox/imu_raw available

Usage:
  source /opt/ros/humble/setup.bash
  source ~/slam/fast_livo2_ws/install/setup.bash   # fast_livo package
  source install/setup.bash                          # ego_planner (imu relay)
  ros2 launch ego_planner fast_livo_sim.launch.py use_rviz:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz", default="true")

    args = [
        DeclareLaunchArgument(
            "use_rviz", default_value=use_rviz,
            description="Launch RViz2 for visualization",
        ),
    ]

    # ---- IMU frame relay (fake Gazebo scoped frame_id -> mid360) ----
    imu_relay_node = Node(
        package="ego_planner",
        executable="imu_frame_relay.py",
        name="imu_frame_relay",
        output="screen",
        arguments=["/livox/imu_raw", "/livox/imu", "mid360"],
    )

    # ---- FAST-LIVO2 SLAM ----
    fast_livo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("fast_livo"),
                "launch",
                "mapping_mid360.launch.py",
            ])
        ),
        launch_arguments={"use_rviz": use_rviz}.items(),
    )

    ld = LaunchDescription()
    for a in args:
        ld.add_action(a)
    ld.add_action(imu_relay_node)
    ld.add_action(fast_livo_launch)

    return ld
