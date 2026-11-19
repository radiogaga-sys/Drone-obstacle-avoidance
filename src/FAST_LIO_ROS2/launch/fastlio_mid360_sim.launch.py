#!/usr/bin/python3
"""
FAST_LIO_ROS2 — complete simulation launch for Livox Mid-360.
Run AFTER starting the simulation (./run_px4_iris_mid360.sh or
./run_iris_mid360_sim.sh).

Nodes launched:
  1. imu_frame_relay    — /livox/imu_raw -> /livox/imu (frame_id=mid360)
  2. fastlio_mapping    — SLAM (Fast-LIO2 LiDAR odometry + mapping)
  3. slam_odom_relay    — /Odometry -> /odom_world (for ego-planner)
  4. rviz2              — visualization (optional)

Data flow:
  libros2_livox.so         -> /mid360 (CustomMsg)
  libgazebo_ros_imu.so     -> /livox/imu_raw (scoped frame)
       │ imu_frame_relay.py
       ▼
  /livox/imu (frame_id=mid360)
       │
  fastlio_mapping ◄──────── /mid360
       │
       ├── /Odometry (camera_init -> body)
       │       │ slam_odom_relay.py
       │       ▼
       │   /odom_world (world -> base_link)  ──→ ego-planner
       │
       ├── /cloud_registered (camera_init)   ──→ ego-planner
       ├── /cloud_registered_body (body)
       ├── /Laser_map
       └── /path

Usage:
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch fast_lio fastlio_mid360_sim.launch.py [use_rviz:=false]
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')

    # Resolve workspace root so we can find top-level scripts.
    # With symlink-install: install/fast_lio/share/fast_lio -> src/FAST_LIO_ROS2
    # Going up 4 levels from the real path hits the workspace root.
    ws_scripts_dir = os.environ.get(
        'FAST_LIO_WS',
        os.path.realpath(os.path.join(package_path, '..', '..', '..', '..'))
    )

    # ── Arguments ──────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    config_file = LaunchConfiguration('config_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo clock (must be true for simulation)'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 for visualization'
    )
    declare_config_file = DeclareLaunchArgument(
        'config_file', default_value='mid360_sim.yaml',
        description='Config file name (in fast_lio/config/)'
    )

    rviz_config = os.path.join(package_path, 'rviz', 'fastlio.rviz')

    # ── 1. IMU frame_id relay ──────────────────────────────────────
    # Gazebo publishes IMU with a scoped frame_id (model::link::sensor).
    # FAST_LIO_ROS2 needs frame_id=mid360 for correct LiDAR-IMU association.
    imu_relay_script = os.path.join(ws_scripts_dir, 'imu_frame_relay.py')
    imu_relay = ExecuteProcess(
        cmd=['python3', imu_relay_script,
             '/livox/imu_raw', '/livox/imu', 'mid360'],
        name='imu_frame_relay',
        output='screen',
    )

    # ── 2. FAST_LIO_ROS2 mapping ───────────────────────────────────
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[PathJoinSubstitution([package_path, 'config', config_file]),
                    {'use_sim_time': use_sim_time}],
    )

    # ── 3. Odometry relay: /Odometry -> /odom_world ────────────────
    odom_relay_script = os.path.join(ws_scripts_dir, 'slam_odom_relay.py')
    odom_relay = ExecuteProcess(
        cmd=['python3', odom_relay_script, 'odom_world', '/Odometry'],
        name='slam_odom_relay',
        output='screen',
    )

    # ── 4. RViz2 ───────────────────────────────────────────────────
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # ── Assemble ───────────────────────────────────────────────────
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_config_file,

        # Relays start immediately; SLAM node after a short delay so
        # the relay nodes have time to register their subscriptions.
        imu_relay,
        odom_relay,
        TimerAction(
            period=1.0,
            actions=[fast_lio_node],
        ),
        rviz_node,
    ])
