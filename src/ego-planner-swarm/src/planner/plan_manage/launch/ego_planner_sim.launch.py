#!/usr/bin/env python3
"""
Unified launch file for FAST-LIVO2 + ego-planner + MAVROS integration.

Nodes started (in staggered order):
  1. MAVROS              — PX4 <-> ROS2 bridge
  2. IMU frame relay     — fixes Gazebo frame_id scoping
  3. FAST-LIVO2 SLAM     — LiDAR-inertial odometry + point clouds
  4. SLAM odom relay     — /aft_mapped_to_init -> /odom_world (frame: world)
  5. ego_planner_node    — replanning FSM + occupancy grid map
  6. traj_server         — B-spline -> PositionCommand @100Hz
  7. cmd_bridge          — PositionCommand -> MAVROS setpoint_raw/local

Prerequisites:
  Terminal 1: ./run_px4_iris_mid360.sh obstacle_forest
  Terminal 2: ros2 launch ego_planner ego_planner_sim.launch.py

Usage:
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch ego_planner ego_planner_sim.launch.py \
    map_size_x:=50.0 map_size_y:=50.0 map_size_z:=5.0
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Map parameters ────────────────────────────────────────────
    map_size_x = LaunchConfiguration("map_size_x", default="30.0")
    map_size_y = LaunchConfiguration("map_size_y", default="30.0")
    map_size_z = LaunchConfiguration("map_size_z", default="5.0")

    # ── Topic remappings ──────────────────────────────────────────
    odom_topic = LaunchConfiguration("odom_topic", default="odom_world")
    cloud_topic = LaunchConfiguration("cloud_topic", default="/cloud_registered")

    # ── Planner tuning ────────────────────────────────────────────
    max_vel = LaunchConfiguration("max_vel", default="2.0")
    max_acc = LaunchConfiguration("max_acc", default="3.0")
    planning_horizon = LaunchConfiguration("planning_horizon", default="7.5")
    flight_type = LaunchConfiguration("flight_type", default="2")
    obs_inflation = LaunchConfiguration("obs_inflation", default="0.2")

    # ── MAVROS ────────────────────────────────────────────────────
    fcu_url = LaunchConfiguration(
        "fcu_url", default="udp://:14540@localhost:14557"
    )

    # ── Goal point (for /goal_point topic) ────────────────────────
    goal_x = LaunchConfiguration("goal_x", default="10.0")
    goal_y = LaunchConfiguration("goal_y", default="0.0")
    goal_z = LaunchConfiguration("goal_z", default="2.0")

    # ── FAST-LIVO2 config ─────────────────────────────────────────
    slam_config = LaunchConfiguration(
        "slam_config",
        default=PathJoinSubstitution([
            FindPackageShare("fast_livo"),
            "config",
            "mid360_sim.yaml",
        ]),
    )

    # =================================================================
    # Declare all arguments
    # =================================================================
    args = [
        DeclareLaunchArgument("map_size_x", default_value=map_size_x),
        DeclareLaunchArgument("map_size_y", default_value=map_size_y),
        DeclareLaunchArgument("map_size_z", default_value=map_size_z),
        DeclareLaunchArgument("odom_topic", default_value=odom_topic),
        DeclareLaunchArgument("cloud_topic", default_value=cloud_topic),
        DeclareLaunchArgument("max_vel", default_value=max_vel),
        DeclareLaunchArgument("max_acc", default_value=max_acc),
        DeclareLaunchArgument("planning_horizon", default_value=planning_horizon),
        DeclareLaunchArgument("flight_type", default_value=flight_type),
        DeclareLaunchArgument("obs_inflation", default_value=obs_inflation),
        DeclareLaunchArgument("fcu_url", default_value=fcu_url),
        DeclareLaunchArgument("goal_x", default_value=goal_x),
        DeclareLaunchArgument("goal_y", default_value=goal_y),
        DeclareLaunchArgument("goal_z", default_value=goal_z),
        DeclareLaunchArgument("slam_config", default_value=slam_config),
    ]

    # =================================================================
    # 1. MAVROS
    # =================================================================
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
        output="screen",
        parameters=[
            {"fcu_url": fcu_url},
            {"gcs_url": ""},
            {"target_system_id": 1},
            {"target_component_id": 1},
        ],
    )

    # =================================================================
    # 2. IMU frame relay (scoped frame_id -> mid360)
    # =================================================================
    imu_relay_node = Node(
        package="ego_planner",
        executable="imu_frame_relay.py",
        name="imu_frame_relay",
        output="screen",
        arguments=["/livox/imu_raw", "/livox/imu", "mid360"],
    )

    # =================================================================
    # 3. FAST-LIVO2 SLAM (via its launch file)
    # =================================================================
    fast_livo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("fast_livo"),
                "launch",
                "mapping_mid360.launch.py",
            ])
        ),
        launch_arguments={
            "use_rviz": "false",
        }.items(),
    )

    # =================================================================
    # 4. SLAM odom relay (camera_init -> world frame)
    # =================================================================
    odom_relay_node = Node(
        package="scouting_bridge",
        executable="slam_odom_relay",
        name="slam_odom_relay",
        output="screen",
        arguments=[odom_topic],
    )

    # =================================================================
    # 5. Ego-planner FSM + occupancy grid
    # =================================================================
    ego_planner_node = Node(
        package="ego_planner",
        executable="ego_planner_node",
        name="ego_planner_node",
        output="screen",
        remappings=[
            ("odom_world", odom_topic),
            ("grid_map/odom", odom_topic),
            ("grid_map/cloud", cloud_topic),
        ],
        parameters=[
            # ── FSM ──
            {"fsm/flight_type": flight_type},
            {"fsm/thresh_replan_time": 1.0},
            {"fsm/thresh_no_replan_meter": 1.0},
            {"fsm/planning_horizon": planning_horizon},
            {"fsm/planning_horizen_time": 3.0},
            {"fsm/emergency_time": 1.0},
            {"fsm/realworld_experiment": False},
            {"fsm/fail_safe": True},
            {"fsm/waypoint_num": 0},
            # ── Grid map ──
            {"grid_map/resolution": 0.2},
            {"grid_map/map_size_x": map_size_x},
            {"grid_map/map_size_y": map_size_y},
            {"grid_map/map_size_z": map_size_z},
            {"grid_map/local_update_range_x": 5.5},
            {"grid_map/local_update_range_y": 5.5},
            {"grid_map/local_update_range_z": 4.5},
            {"grid_map/obstacles_inflation": obs_inflation},
            {"grid_map/local_map_margin": 10},
            {"grid_map/ground_height": -0.01},
            {"grid_map/use_depth_filter": False},
            {"grid_map/cx": 321.04638671875},
            {"grid_map/cy": 243.44969177246094},
            {"grid_map/fx": 387.229248046875},
            {"grid_map/fy": 387.229248046875},
            {"grid_map/p_hit": 0.70},
            {"grid_map/p_miss": 0.35},
            {"grid_map/p_min": 0.12},
            {"grid_map/p_max": 0.90},
            {"grid_map/p_occ": 0.80},
            {"grid_map/min_ray_length": 0.3},
            {"grid_map/max_ray_length": 10.0},
            {"grid_map/virtual_ceil_height": 4.9},
            {"grid_map/visualization_truncate_height": 1.8},
            {"grid_map/show_occ_time": False},
            {"grid_map/pose_type": 2},
            {"grid_map/frame_id": "world"},
            # ── Planner manager ──
            {"manager/max_vel": max_vel},
            {"manager/max_acc": max_acc},
            {"manager/max_jerk": 4.0},
            {"manager/control_points_distance": 0.4},
            {"manager/feasibility_tolerance": 0.05},
            {"manager/planning_horizon": planning_horizon},
            {"manager/use_distinctive_trajs": True},
            {"manager/drone_id": 0},
            # ── Optimization ──
            {"optimization/lambda_smooth": 1.0},
            {"optimization/lambda_collision": 0.5},
            {"optimization/lambda_feasibility": 0.1},
            {"optimization/lambda_fitness": 1.0},
            {"optimization/dist0": 0.5},
            {"optimization/swarm_clearance": 0.5},
            {"optimization/max_vel": max_vel},
            {"optimization/max_acc": max_acc},
            # ── B-Spline limits ──
            {"bspline/limit_vel": max_vel},
            {"bspline/limit_acc": max_acc},
            {"bspline/limit_ratio": 1.1},
            # ── No dynamic obstacle prediction ──
            {"prediction/obj_num": 0},
        ],
    )

    # =================================================================
    # 6. Trajectory server (B-spline -> PositionCommand @100Hz)
    # =================================================================
    traj_server_node = Node(
        package="ego_planner",
        executable="traj_server",
        name="traj_server",
        output="screen",
        parameters=[
            {"traj_server/time_forward": 1.0},
        ],
    )

    # =================================================================
    # 7. Command bridge (PositionCommand -> MAVROS /setpoint_raw/local)
    # =================================================================
    cmd_bridge_node = Node(
        package="ego_planner",
        executable="cmd_bridge.py",
        name="cmd_bridge",
        output="screen",
    )

    # =================================================================
    # Assemble with staggered timing
    # =================================================================
    ld = LaunchDescription()
    for a in args:
        ld.add_action(a)

    # t=0s: MAVROS + IMU relay (independent)
    ld.add_action(mavros_node)
    ld.add_action(imu_relay_node)

    # t=3s: FAST-LIVO2 (needs IMU relay running, LiDAR topic ready)
    ld.add_action(TimerAction(period=3.0, actions=[fast_livo_launch]))

    # t=5s: Odom relay (needs /aft_mapped_to_init from SLAM)
    ld.add_action(TimerAction(period=5.0, actions=[odom_relay_node]))

    # t=6s: Ego-planner (needs odom_world + cloud_registered)
    ld.add_action(TimerAction(period=6.0, actions=[ego_planner_node]))

    # t=7s: Traj server + cmd bridge
    ld.add_action(TimerAction(period=7.0, actions=[traj_server_node]))
    ld.add_action(TimerAction(period=7.0, actions=[cmd_bridge_node]))

    return ld
