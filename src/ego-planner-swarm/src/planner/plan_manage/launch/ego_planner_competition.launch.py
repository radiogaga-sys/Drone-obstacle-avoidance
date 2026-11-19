#!/usr/bin/env python3
"""
Competition launch for ego-planner with FAST_LIO_ROS2.

Strategy:
  - B stage: Strategy 1 (pass-through only, no reconnaissance)
  - D stage: Fixed ring (100 pts)
  - Unified parameters for all stages (no runtime switching)

Waypoint flow (PRESET_TARGET mode):
  - All waypoints are pre-loaded from competition_waypoints.yaml as FSM parameters
  - FSM reads waypoints at startup, waits for /traj_start_trigger
  - After trigger, FSM auto-advances through all waypoints sequentially
  - mission_controller_competition.py sends trigger and monitors progress

Key differences from ego_planner_fast_lio2.launch.py:
  - flight_type=2 (PRESET_TARGET): sequential waypoint execution
  - resolution=0.12m: fine enough for A-stage narrow gap
  - obstacles_inflation=0.24m: 2-voxel inflation (0.24m at 0.12m res)
  - dist0=0.45m: NUC-tuned safety margin
  - max_vel=0.5m/s, max_acc=2.0m/s^2: conservative dynamics
  - cloud_point_stride=1: full point cloud
  - max_ray_length=7.0m: reduced sensing range

Usage:
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch ego_planner ego_planner_competition.launch.py
"""
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_waypoints(yaml_path: str) -> list:
    """
    Read competition_waypoints.yaml and flatten all stage waypoints
    into a single list of [x, y, z].

    Returns list of dicts: [{"x": ..., "y": ..., "z": ..., "stage": "A"|"B"|"C"|"D"}, ...]
    """
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    waypoints = []
    for stage_name in ['A', 'B', 'C', 'D']:
        stage = data['stages'][stage_name]
        for wp in stage['waypoints']:
            waypoints.append({
                'x': float(wp[0]),
                'y': float(wp[1]),
                'z': float(wp[2]),
                'yaw': float(wp[3]) if len(wp) > 3 else float('nan'),
                'stage': stage_name,
            })
    return waypoints


def create_ego_planner_node(yaml_path=None, remap_overrides=None, param_overrides=None):
    """
    Create ego_planner_node with competition defaults.

    All planner parameters are set to conservative competition values.
    Override them by passing param_overrides — appended after defaults
    so the override value takes effect (ROS 2 last-wins).

    Args:
        yaml_path: Path to competition_waypoints.yaml (for PRESET_TARGET waypoints).
        remap_overrides: Dict of {from: to} topic remap overrides.
        param_overrides: Dict of {param_name: value} appended after defaults.

    Returns:
        Node: Configured ego_planner_node.
    """
    # ── Load waypoints ──────────────────────────────────────────
    waypoints = _load_waypoints(yaml_path) if yaml_path and os.path.exists(yaml_path) else []
    waypoint_num = len(waypoints)

    # ── FSM parameters ───────────────────────────────────────────
    fsm_params = [
        {"fsm/flight_type": 2},
        {"fsm/thresh_replan_time": 1.0},
        {"fsm/thresh_no_replan_meter": 0.5},
        {"fsm/planning_horizon": 7.5},
        {"fsm/planning_horizen_time": 3.0},
        {"fsm/emergency_time": 1.0},
        {"fsm/realworld_experiment": True},
        {"fsm/fail_safe": True},
        {"fsm/waypoint_num": waypoint_num},
    ]
    for i, wp in enumerate(waypoints):
        fsm_params.append({f"fsm/waypoint{i}_x": wp['x']})
        fsm_params.append({f"fsm/waypoint{i}_y": wp['y']})
        fsm_params.append({f"fsm/waypoint{i}_z": wp['z']})
        fsm_params.append({f"fsm/waypoint{i}_yaw": wp['yaw']})

    # ── Default parameters (competition values) ──────────────────
    params = [
        *fsm_params,
        # Grid map
        {"grid_map/resolution": 0.12},
        {"grid_map/map_size_x": 30.0},
        {"grid_map/map_size_y": 30.0},
        {"grid_map/map_size_z": 5.0},
        {"grid_map/local_update_range_x": 5.5},
        {"grid_map/local_update_range_y": 5.5},
        {"grid_map/local_update_range_z": 4.5},
        {"grid_map/odom_depth_timeout": 3.0},
        {"grid_map/obstacles_inflation": 0.24},
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
        {"grid_map/min_ray_length": 0.15},
        {"grid_map/max_ray_length": 7.0},
        {"grid_map/virtual_ceil_height":2.0},
        {"grid_map/visualization_truncate_height": 1.8},
        {"grid_map/show_occ_time": False},
        {"grid_map/pose_type": 2},
        {"grid_map/frame_id": "camera_init"},
        {"grid_map/cloud_point_stride": 1},
        # Planner manager
        {"manager/max_vel": 0.5},
        {"manager/max_acc": 2.0},
        {"manager/max_jerk": 2.0},
        {"manager/control_points_distance": 0.3},
        {"manager/feasibility_tolerance": 0.05},
        {"manager/planning_horizon": 7.5},
        {"manager/use_distinctive_trajs": True},
        {"manager/drone_id": 0},
        # Optimization
        {"optimization/lambda_smooth": 1.0},
        {"optimization/lambda_collision": 1.0},
        {"optimization/lambda_feasibility": 0.1},
        {"optimization/lambda_fitness": 1.0},
        {"optimization/dist0": 0.45},
        {"optimization/swarm_clearance": 0.6},
        {"optimization/max_vel": 0.5},
        {"optimization/max_acc": 2.0},
        # B-Spline
        {"bspline/limit_vel": 0.5},
        {"bspline/limit_acc": 2.0},
        {"bspline/limit_ratio": 1.1},
        # Object prediction
        {"prediction/obj_num": 0},
    ]

    # ── Apply param overrides (appended → last-wins) ─────────────
    if param_overrides:
        for k, v in param_overrides.items():
            params.append({k: v})

    # ── Build remappings ──────────────────────────────────────────
    _default_remaps = {
        "odom_world": "/Odometry",
        "grid_map/odom": "/Odometry",
        "grid_map/depth": "/mid360_PointCloud2",
        "grid_map/cloud": "/mid360_PointCloud2",
        "move_base_simple/goal": "/goal_point",
    }
    if remap_overrides:
        _default_remaps.update(remap_overrides)

    return Node(
        package="ego_planner",
        executable="ego_planner_node",
        name="ego_planner_node",
        output="screen",
        remappings=[(k, v) for k, v in _default_remaps.items()],
        parameters=params,
    )


def generate_launch_description():
    # ── Resolve waypoints yaml path ─────────────────────────────
    # Try workspace-relative path (development), fallback to package share
    _launch_dir = os.path.dirname(os.path.realpath(__file__))
    _ws_yaml = os.path.abspath(os.path.join(
        _launch_dir, '..', '..', '..', '..', 'config', 'competition_waypoints.yaml'))
    if os.path.exists(_ws_yaml):
        _default_yaml = _ws_yaml
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            _default_yaml = os.path.join(
                get_package_share_directory('ego_planner'), 'config', 'competition_waypoints.yaml')
        except Exception:
            _default_yaml = ''

    waypoints_yaml = LaunchConfiguration("waypoints_yaml", default=_default_yaml)

    # ── Map size ─────────────────────────────────────────────────
    map_size_x = LaunchConfiguration("map_size_x", default="30.0")
    map_size_y = LaunchConfiguration("map_size_y", default="30.0")
    map_size_z = LaunchConfiguration("map_size_z", default="5.0")

    # ── Topics (FAST_LIO_ROS2 defaults) ──────────────────────────
    odom_topic = LaunchConfiguration("odom_topic", default="/odom_world")
    cloud_topic = LaunchConfiguration("cloud_topic", default="/cloud_registered")
    depth_topic = LaunchConfiguration("depth_topic", default="/cloud_registered_body")

    # ── Planner tuning (unified for all competition stages) ──────
    max_vel = LaunchConfiguration("max_vel", default="0.5")
    max_acc = LaunchConfiguration("max_acc", default="2.0")
    planning_horizon = LaunchConfiguration("planning_horizon", default="7.5")
    flight_type = LaunchConfiguration("flight_type", default="2")
    obs_inflation = LaunchConfiguration("obs_inflation", default="0.24")

    # ── Arguments ────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("waypoints_yaml", default_value=_default_yaml,
                              description="Path to competition_waypoints.yaml"),
        DeclareLaunchArgument("map_size_x", default_value=map_size_x),
        DeclareLaunchArgument("map_size_y", default_value=map_size_y),
        DeclareLaunchArgument("map_size_z", default_value=map_size_z),
        DeclareLaunchArgument("odom_topic", default_value=odom_topic),
        DeclareLaunchArgument("cloud_topic", default_value=cloud_topic),
        DeclareLaunchArgument("depth_topic", default_value=depth_topic,
                              description="PointCloud2 topic for grid_map/depth (body-frame for USE_LIDAR_CLOUD)"),
        DeclareLaunchArgument("max_vel", default_value=max_vel),
        DeclareLaunchArgument("max_acc", default_value=max_acc),
        DeclareLaunchArgument("planning_horizon", default_value=planning_horizon),
        DeclareLaunchArgument("flight_type", default_value=flight_type),
        DeclareLaunchArgument("obs_inflation", default_value=obs_inflation),
    ]

    # ── 1. Ego-planner FSM + occupancy grid ──────────────────────
    ego_planner_node = create_ego_planner_node(
        yaml_path=_default_yaml,
        remap_overrides={
            "odom_world": odom_topic,
            "grid_map/odom": odom_topic,
            "grid_map/depth": depth_topic,
            "grid_map/cloud": cloud_topic,
        },
        param_overrides={
            "fsm/flight_type": flight_type,
            "fsm/planning_horizon": planning_horizon,
            "grid_map/map_size_x": map_size_x,
            "grid_map/map_size_y": map_size_y,
            "grid_map/map_size_z": map_size_z,
            "grid_map/obstacles_inflation": obs_inflation,
            "manager/max_vel": max_vel,
            "manager/max_acc": max_acc,
            "manager/planning_horizon": planning_horizon,
            "optimization/max_vel": max_vel,
            "optimization/max_acc": max_acc,
            "bspline/limit_vel": max_vel,
            "bspline/limit_acc": max_acc,
        },
    )

    # ── 2. Trajectory server ─────────────────────────────────────
    traj_server_node = Node(
        package="ego_planner",
        executable="traj_server",
        name="traj_server",
        output="screen",
        parameters=[
            {"traj_server/time_forward": 1.0},
        ],
    )

    # ── 3. Command bridge (C++, replaces cmd_bridge.py) ──────────
    cmd_bridge_node = Node(
        package="ego_planner",
        executable="cmd_bridge",
        name="cmd_bridge",
        output="screen",
    )

    # ── Assemble ─────────────────────────────────────────────────
    ld = LaunchDescription()
    for a in args:
        ld.add_action(a)

    ld.add_action(TimerAction(period=1.0, actions=[ego_planner_node]))
    ld.add_action(TimerAction(period=2.0, actions=[traj_server_node]))
    ld.add_action(TimerAction(period=2.0, actions=[cmd_bridge_node]))

    return ld
