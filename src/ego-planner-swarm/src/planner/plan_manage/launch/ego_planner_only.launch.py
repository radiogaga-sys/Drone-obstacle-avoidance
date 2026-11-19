#!/usr/bin/env python3
"""
Launch ego-planner with all bridge nodes (MAVROS and SLAM must be running).

Starts:
  1. SLAM odom relay  — /aft_mapped_to_init -> /odom_world (frame: world)
  2. ego_planner_node  — replanning FSM + occupancy grid map
  3. traj_server       — B-spline -> PositionCommand @100Hz
  4. cmd_bridge        — PositionCommand -> MAVROS /setpoint_raw/local

Prerequisites:
  - MAVROS running (ros2 run mavros mavros_node ...)
  - FAST-LIVO2 running (ros2 launch ego_planner fast_livo_sim.launch.py)

Usage:
  source /opt/ros/humble/setup.bash
  source ~/slam/fast_livo2_ws/install/setup.bash
  source install/setup.bash
  ros2 launch ego_planner ego_planner_only.launch.py \
    map_size_x:=50.0 map_size_y:=50.0 map_size_z:=5.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Map size ─────────────────────────────────────────────────
    map_size_x = LaunchConfiguration("map_size_x", default="100.0")
    map_size_y = LaunchConfiguration("map_size_y", default="10.0")
    map_size_z = LaunchConfiguration("map_size_z", default="5.0")

    # ── Topics ───────────────────────────────────────────────────
    odom_topic = LaunchConfiguration("odom_topic", default="odom_world")
    cloud_topic = LaunchConfiguration("cloud_topic", default="/cloud_registered")

    # ── Planner tuning ───────────────────────────────────────────
    max_vel = LaunchConfiguration("max_vel", default="2.0")
    max_acc = LaunchConfiguration("max_acc", default="3.0")
    planning_horizon = LaunchConfiguration("planning_horizon", default="7.5")
    flight_type = LaunchConfiguration("flight_type", default="1")
    obs_inflation = LaunchConfiguration("obs_inflation", default="0.2")

    # ── Arguments ────────────────────────────────────────────────
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
    ]

    # ── 1. SLAM odom relay ───────────────────────────────────────
    odom_relay_node = Node(
        package="scouting_bridge",
        executable="slam_odom_relay",
        name="slam_odom_relay",
        output="screen",
        arguments=[odom_topic],
    )

    # ── 2. Ego-planner FSM + occupancy grid ──────────────────────
    ego_planner_node = Node(
        package="ego_planner",
        executable="ego_planner_node",
        name="ego_planner_node",
        output="screen",
        remappings=[
            ("odom_world", odom_topic),
            ("grid_map/odom", odom_topic),
            ("grid_map/depth", cloud_topic),
            ("grid_map/cloud", cloud_topic),
            ("move_base_simple/goal", "/goal_point"),
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
            {"grid_map/resolution": 0.3},
            {"grid_map/map_size_x": map_size_x},
            {"grid_map/map_size_y": map_size_y},
            {"grid_map/map_size_z": map_size_z},
            {"grid_map/local_update_range_x": 5.5},
            {"grid_map/local_update_range_y": 5.5},
            {"grid_map/local_update_range_z": 4.5},
            {"grid_map/obstacles_inflation": obs_inflation},
            {"grid_map/local_map_margin": 7},
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
            {"grid_map/cloud_point_stride": 2},
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
            # ── B-Spline ──
            {"bspline/limit_vel": max_vel},
            {"bspline/limit_acc": max_acc},
            {"bspline/limit_ratio": 1.1},
            # ── Object prediction ──
            {"prediction/obj_num": 0},
        ],
    )

    # ── 3. Trajectory server ─────────────────────────────────────
    traj_server_node = Node(
        package="ego_planner",
        executable="traj_server",
        name="traj_server",
        output="screen",
        parameters=[
            {"traj_server/time_forward": 1.0},
        ],
    )

    # ── 4. Command bridge (C++, replaces cmd_bridge.py) ────────────
    cmd_bridge_node = Node(
        package="ego_planner",
        executable="cmd_bridge",
        name="cmd_bridge",
        output="screen",
    )

    # ── Assemble (wait for SLAM topics to be ready) ──────────────
    ld = LaunchDescription()
    for a in args:
        ld.add_action(a)

    ld.add_action(odom_relay_node)
    ld.add_action(TimerAction(period=1.0, actions=[ego_planner_node]))
    ld.add_action(TimerAction(period=2.0, actions=[traj_server_node]))
    ld.add_action(TimerAction(period=2.0, actions=[cmd_bridge_node]))

    return ld
