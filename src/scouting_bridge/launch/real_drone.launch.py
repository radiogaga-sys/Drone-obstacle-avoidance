#!/usr/bin/python3
"""
One-click launch for real drone with Mid-360 LiDAR.

Nodes (launched in dependency order with staggered delays):
  1. livox_ros_driver2       - Mid-360 LiDAR driver
  2. FAST_LIO_ROS2           - LIO SLAM
  3. vision_pose_relay       - /Odometry -> /mavros/vision_pose/pose
  4. slam_odom_relay         - /Odometry -> /odom_world
  5. tf_broadcaster          - body->base_link, odom->base_link, base_link->mid360/camera_down
  6. usb_cam_down            - USB downward camera driver (optional: enable_apriltag)
  7. apriltag_detector       - AprilTag detection (optional: enable_apriltag)
  8. tag_pose_relay          - TF tag pose -> world PoseStamped (optional: enable_apriltag)
  9. odom_visualization      - /Odometry -> /robot MESH_RESOURCE for RViz
 10. ego_planner_node        - FSM + occupancy grid
 11. traj_server             - B-spline trajectory server
 12. cmd_bridge (C++)        - /position_cmd -> /mavros/setpoint_raw/local
 13. mission_controller      - competition mission (optional)
 14. tag_mission_controller  - post-competition scan & landing (optional)

Usage:
  ros2 launch scouting_bridge real_drone.launch.py
  ros2 launch scouting_bridge real_drone.launch.py enable_mission:=false
  ros2 launch scouting_bridge real_drone.launch.py enable_apriltag:=false enable_tag_mission:=false
  ros2 launch scouting_bridge real_drone.launch.py lidar_frame:=mid360

Override TF offsets via the bridge config file:
  src/scouting_bridge/config/real_drone.yaml
"""

import os
import importlib.util

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # --- Arguments -------------------------------------------------
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="false",
        description="Launch RViz2 (off on real drone by default)")

    mission_arg = DeclareLaunchArgument(
        "enable_mission", default_value="true",
        description="Start competition mission controller")

    apriltag_arg = DeclareLaunchArgument(
        "enable_apriltag", default_value="true",
        description="Start AprilTag detection pipeline (camera + detector)")

    tag_mission_arg = DeclareLaunchArgument(
        "enable_tag_mission", default_value="true",
        description="Start tag scanning & landing mission controller")

    lidar_frame_arg = DeclareLaunchArgument(
        "lidar_frame", default_value="livox_frame",
        description="LiDAR point cloud frame_id")

    enable_rviz = LaunchConfiguration("rviz")
    enable_mission = LaunchConfiguration("enable_mission")
    enable_apriltag = LaunchConfiguration("enable_apriltag")
    enable_tag_mission = LaunchConfiguration("enable_tag_mission")
    lidar_frame = LaunchConfiguration("lidar_frame")

    # --- Paths -----------------------------------------------------
    pkg_scouting = get_package_share_directory("scouting_bridge")
    bridge_config = os.path.join(pkg_scouting, "config", "real_drone.yaml")
    mid360_config = os.path.join(
        get_package_share_directory("fast_lio"), "config", "mid360.yaml"
    )
    rviz_config = os.path.join(
        get_package_share_directory("fast_lio"), "rviz", "fastlio.rviz")

    # AprilTag pipeline configs
    apriltag_config = os.path.join(pkg_scouting, "config", "apriltag_config.yaml")
    usb_cam_config = os.path.join(pkg_scouting, "config", "usb_cam_down.yaml")

    # ==============================================================
    # 1. Mid-360 LiDAR driver
    # ==============================================================
    lidar_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[{
            "frame_id": lidar_frame,
            "xfer_format": 1,
            "multi_topic": 0,
            "data_src": 0,
            "publish_freq": 10.0,
            "output_data_type": 0,
            "lvx_file_path": "/home/d406/livox_test.lvx",
            "cmdline_input_bd_code": "livox0000000001",
            "user_config_path": os.path.join(
                get_package_share_directory("livox_ros_driver2"),
                "config", "MID360s_config.json",
            ),
        }],
    )

    # ==============================================================
    # 2. FAST_LIO_ROS2 SLAM
    # ==============================================================
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="laser_mapping",
        output="screen",
        parameters=[mid360_config],
        # respawn deliberately OFF (was True + respawn_delay=2.0): FAST_LIO
        # fixes its world origin (camera_init) at process start, so a mid-flight
        # restart would move the origin to the drone's current position --
        # /odom_world jumps to ~0, ego-planner replans from a bogus pose, and
        # vision_pose_relay forwards that jump into PX4 EKF2. launch's respawn
        # ignores the exit code, so any crash triggered it. With respawn off a
        # crash just hovers in place (traj_server "hover when finish traj_").
        respawn=False,
    )

    # ==============================================================
    # 3. scouting_bridge glue scripts (all params in real_drone.yaml)
    # ==============================================================
    vision_pose_relay_node = Node(
        package="scouting_bridge",
        executable="vision_pose_relay",
        name="vision_pose_relay",
        output="screen",
        parameters=[bridge_config],
    )

    slam_odom_relay_node = Node(
        package="scouting_bridge",
        executable="slam_odom_relay",
        name="slam_odom_relay",
        output="screen",
        parameters=[bridge_config],
    )

    tf_broadcaster_node = Node(
        package="scouting_bridge",
        executable="tf_broadcaster",
        name="tf_broadcaster",
        output="screen",
        parameters=[bridge_config],
    )

    # ==============================================================
    # 4. AprilTag detection pipeline (raw image → detector → pose relay)
    #    No image_proc: apriltag uses raw image + original K+D from
    #    usb_cam; OpenCV solvePnP handles undistortion internally.
    # ==============================================================
    usb_cam_node = Node(
        condition=IfCondition(enable_apriltag),
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_down",
        namespace="camera_down",
        output="screen",
        parameters=[usb_cam_config],
    )

    apriltag_node = Node(
        condition=IfCondition(enable_apriltag),
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

    tag_pose_relay_node = Node(
        condition=IfCondition(enable_apriltag),
        package="scouting_bridge",
        executable="tag_pose_relay",
        name="tag_pose_relay",
        output="screen",
        parameters=[bridge_config],
    )

    # ==============================================================
    # 4.5 Drone model visualization (odom -> MESH_RESOURCE marker)
    # ==============================================================
    odom_vis_node = Node(
        package="odom_visualization",
        executable="odom_visualization",
        name="odom_visualization",
        output="screen",
        parameters=[{
            "mesh_resource": "package://odom_visualization/meshes/hummingbird.mesh",
            "color/r": 0.2,
            "color/g": 0.6,
            "color/b": 0.9,
            "color/a": 0.7,
            "robot_scale": 1.0,
            "frame_id": "camera_init",
            "tf45": False,
            "covariance_position": False,
            "covariance_velocity": False,
        }],
        remappings=[("odom", "/Odometry")],
    )

    # ==============================================================
    # 5. ego-planner (competition defaults + real-drone overrides)
    # ==============================================================
    # Load ego_planner_node from competition launch — inherits FSM
    # params, waypoints, camera intrinsics, and conservative tuning.
    # Only map size and topic remappings are overridden for real drone.
    _ego_planner_share = get_package_share_directory("ego_planner")
    _competition_launch = os.path.join(
        _ego_planner_share, "launch", "ego_planner_competition.launch.py")
    _spec = importlib.util.spec_from_file_location(
        "ego_planner_competition", _competition_launch)
    _competition_mod = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_competition_mod)

    # YAML path: prefer workspace-relative, fallback to package share
    _yaml_ws = os.path.abspath(os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        "..", "..", "ego-planner-swarm", "config", "competition_waypoints.yaml"))
    _yaml_path = _yaml_ws if os.path.exists(_yaml_ws) else os.path.join(
        _ego_planner_share, "config", "competition_waypoints.yaml")

    ego_planner_node = _competition_mod.create_ego_planner_node(
        yaml_path=_yaml_path,
        remap_overrides={
            "odom_world": "/odom_world",
            "grid_map/odom": "/odom_world",
            "grid_map/cloud": "/cloud_registered",
            "grid_map/depth": "/cloud_registered_body",
        },
        param_overrides={
            # FSM
            "fsm/flight_type": 2,
            "fsm/planning_horizon": 7.5,
            # Grid map
            "grid_map/map_size_x": 40.0,
            "grid_map/map_size_y": 40.0,
            "grid_map/map_size_z": 5.0,
            "grid_map/obstacles_inflation": 0.24,
            "grid_map/resolution": 0.08,
            "optimization/dist0": 0.45,
            # Dynamics (synchronized across manager / optimization / bspline)
            "manager/max_vel": 0.8,
            "manager/max_acc": 2.0,
            "manager/planning_horizon": 7.5,
            "optimization/max_vel": 0.8,
            "optimization/max_acc": 2.0,
            "bspline/limit_vel": 0.8,
            "bspline/limit_acc": 2.0,
        },
    )

    traj_server_node = Node(
        package="ego_planner",
        executable="traj_server",
        name="traj_server",
        output="screen",
        parameters=[{"traj_server/time_forward": 1.0}],
    )

    cmd_bridge_node = Node(
        package="ego_planner",
        executable="cmd_bridge",
        name="cmd_bridge",
        output="screen",
    )

    # ==============================================================
    # 6. Competition mission controller (optional)
    # ==============================================================
    mission_node = Node(
        condition=IfCondition(enable_mission),
        package="ego_planner",
        executable="mission_controller_competition.py",
        name="mission_controller",
        output="screen",
        parameters=[{
            "mission/max_flight_height": 3.0,
            "mission/navigation_height": 1.5,
            "mission/takeoff_height": 1.0,
            "mission/landing_height": -1.5,
            "mission/min_land_interval": 0.2,
            "mission/max_task_num": 9,
        }],
    )

    # ==============================================================
    # 7. Tag scanning & landing mission controller (optional)
    #    Waits for /mission/stage = "complete" then takes over.
    # ==============================================================
    tag_mission_node = Node(
        condition=IfCondition(enable_tag_mission),
        package="scouting_bridge",
        executable="tag_mission_controller",
        name="tag_mission_controller",
        output="screen",
        parameters=[{
            "auto_start": False,           # Wait for /mission/stage = "complete"
            "target_tag_id": 0,            # 0 = any tag, or specific ID
            "scan_timeout": 30.0,          # Seconds to wait for tag detection
            "max_retries": 3,              # Retry limit before RTL
            "retry_climb": 1.0,            # Extra height per retry (m)
            "descend_step": 0.5,           # Step size per descent level (m)
            "land_height": 0.3,            # Height to trigger disarm (m)
            "pid_kp_xy": 0.5,
            "pid_ki_xy": 0.02,
            "pid_kd_xy": 0.15,
            "max_horizontal_vel": 1.0,
            "auto_land": True,
        }],
    )

    # ==============================================================
    # RViz2 (off by default on real drone)
    # ==============================================================
    rviz_node = Node(
        condition=IfCondition(enable_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    # ==============================================================
    # Assembly with startup delays
    # ==============================================================
    ld = LaunchDescription([
        rviz_arg,
        mission_arg,
        apriltag_arg,
        tag_mission_arg,
        lidar_frame_arg,
    ])

    # Timing rationale:
    # - Livox driver has a built-in 3s sleep before polling data, so all
    #   downstream nodes (FAST_LIO, bridge scripts) can start at T=0 safely.
    # - Bridge scripts only create subscribers -- harmless with no publisher yet.
    # - tf_broadcaster publishes static TFs in constructor -- start early.
    # - Camera + AprilTag pipeline starts early so it's ready when needed.
    # - ego-planner starts at T=2s to reduce console noise ("no odom." spam),
    #   but its INIT state handles missing odom gracefully regardless.
    ld.add_action(lidar_node)
    ld.add_action(fast_lio_node)
    ld.add_action(vision_pose_relay_node)
    ld.add_action(slam_odom_relay_node)
    ld.add_action(tf_broadcaster_node)

    # AprilTag pipeline (T=1s -- after TF tree is established)
    ld.add_action(TimerAction(period=1.0, actions=[usb_cam_node]))
    ld.add_action(TimerAction(period=1.0, actions=[apriltag_node]))
    ld.add_action(TimerAction(period=1.0, actions=[tag_pose_relay_node]))
    # Drone model visualization (also T=1s, independent of AprilTag)
    ld.add_action(TimerAction(period=1.0, actions=[odom_vis_node]))

    # ego-planner
    ld.add_action(TimerAction(period=2.0, actions=[ego_planner_node]))
    ld.add_action(TimerAction(period=2.5, actions=[traj_server_node]))
    ld.add_action(TimerAction(period=3.0, actions=[cmd_bridge_node]))

    # Mission controllers
    ld.add_action(TimerAction(period=3.5, actions=[mission_node]))
    ld.add_action(TimerAction(period=4.0, actions=[tag_mission_node]))

    ld.add_action(rviz_node)

    return ld
