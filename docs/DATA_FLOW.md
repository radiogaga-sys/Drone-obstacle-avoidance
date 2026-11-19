# UAV 避障系统数据流整理

> 工作空间：`/home/pangyu/Projects/slam/UAV`
> 整理范围：`src/` 下 6 个包 —— `scouting_bridge`、`ego-planner-swarm`、`FAST_LIO_ROS2`、`livox_ros_driver2`、`apriltag_ros`、`usb_cam`
> 平台：ROS 2 Humble + PX4/MAVROS + Livox Mid-360
>
> 📊 配套可交互数据流图：[`docs/UAV_DATAFLOW.html`](UAV_DATAFLOW.html)（浏览器打开，支持主题切换 / 缩放 / 搜索 / 视图聚焦）
> 图纸源码：[`docs/archify/uav-dataflow.dataflow.json`](archify/uav-dataflow.dataflow.json)
> 🚀 新手上手 / "我要改 X 去哪改"：[`docs/GETTING_STARTED.md`](GETTING_STARTED.md)

---

## 0. 一句话总览

这是一套 **"激光雷达 LIO 定位 + 外部视觉位姿喂给 PX4 EKF2 + ego-planner 局部避障 + MAVROS Offboard 位置控制"** 的真机自主飞行系统；
`scouting_bridge` 是胶水包，负责把 **SLAM 的坐标系/话题** 翻译成 **PX4 和 ego-planner 各自期望的格式**，并在比赛任务结束后把控制权从 `cmd_bridge` 交接给 AprilTag 精准降落控制器。

系统里有 **两条独立的闭环**，这是理解整个数据流的关键：

| 闭环 | 目的 | 链路 | 事实上的世界系 |
|---|---|---|---|
| **A. 定位闭环** | 让 PX4 知道"我在哪" | FAST_LIO `/Odometry` → `vision_pose_relay` → `/mavros/vision_pose/pose` → PX4 EKF2 → `/mavros/local_position/odom` | `camera_init`（SLAM 上电原点） |
| **B. 控制闭环** | 让无人机按规划飞行 | `/Odometry` → `slam_odom_relay` → `/odom_world` → ego-planner → `planning/bspline` → `traj_server` → `/position_cmd` → `cmd_bridge` → `/mavros/setpoint_raw/local` → PX4 | `world`（数值上等于 `camera_init`） |

**关键约定：`camera_init`、`world`、`odom` 三者在数值上被视为同一个原点**（`slam_odom_relay` 只改 `frame_id` 字符串、不做任何坐标变换）。整个系统依赖"SLAM 上电位置 = PX4 本地原点"这个假设——**这是一个没有任何代码强制或校验的假设，也是最容易被忽略的飞行风险，见 R10 / §8.1**。

---

## 1. 启动链与时间线

### 1.1 启动顺序（用户当前做法）

```bash
# 终端 1：先起 MAVROS（连接 PX4 飞控）—— 不在本仓库内，按飞控侧既有方式启动

# 终端 2：一键起所有（SLAM + 桥接 + 相机 + 规划 + 任务）
ros2 launch scouting_bridge real_drone.launch.py
```

MAVROS 是本系统与 PX4 之间**唯一的接口**，负责 `/mavros/*` 的全部话题与服务；本仓库不提供它的启动文件。
启动前请确认 `fcu_url` 指向真机链路而非 SITL 回环（`udp://127.0.0.1:14552@127.0.0.1:14552` 只在仿真下有效），见 §8 风险 R1。

### 1.2 `real_drone.launch.py` 的启动时间线

启动参数：`rviz`(false) / `enable_mission`(true) / `enable_apriltag`(true) / `enable_tag_mission`(true) / `lidar_frame`(livox_frame)

| T (s) | 节点 | 包 | 备注 |
|---|---|---|---|
| 0.0 | `livox_lidar_publisher` | livox_ros_driver2 | 驱动内部自带 ~3s 启动延时才出数据 |
| 0.0 | `laser_mapping` (FAST_LIO) | fast_lio | `respawn=False`（刻意不自动重启，见 §3.2） |
| 0.0 | `vision_pose_relay` | scouting_bridge | 只建订阅，无数据也无害 |
| 0.0 | `slam_odom_relay` | scouting_bridge | 同上 |
| 0.0 | `tf_broadcaster` | scouting_bridge | 构造函数里立刻发静态 TF |
| 1.0 | `usb_cam_down` | usb_cam | namespace `camera_down` |
| 1.0 | `apriltag_detector` | apriltag_ros | |
| 1.0 | `tag_pose_relay` | scouting_bridge | |
| 1.0 | `odom_visualization` | odom_visualization | 仅 RViz 可视化 |
| 2.0 | `ego_planner_node` | ego_planner | 构造时会**阻塞等待** `/Odometry` + `/traj_start_trigger` |
| 2.5 | `traj_server` | ego_planner | |
| 3.0 | `cmd_bridge` | ego_planner (C++) | |
| 3.5 | `mission_controller` | ego_planner (Python) | 起飞 + 触发 + 监控 |
| 4.0 | `tag_mission_controller` | scouting_bridge | 等 `/mission/stage == "complete"` |

> `real_drone_no_tag.launch.py` 是同一份脚本去掉 AprilTag 链路（无 camera/apriltag/tag_pose_relay/tag_mission_controller）的版本。
> `apriltag_pipeline.launch.py` 是**独立调试**用的相机链路，已与主链路统一：同样走 `raw_mjpeg` → `/camera_down/image_raw/compressed`，无 `image_proc`，`image_rect` / `camera_info` 的 remap 与 `real_drone.launch.py` 完全一致；默认设备与分辨率也对齐 `config/usb_cam_down.yaml`（`/dev/video0`、1280×720、30 fps）。
> ⚠️ 覆盖 `image_width` / `image_height` **不会**自动缩放 `config/camera_down.yaml` 的内参，非标定分辨率下 tag 位姿不可信。

---

## 2. 分层数据流总图

```mermaid
flowchart LR
  subgraph L0["① 传感器层"]
    LIDAR["livox_lidar_publisher<br/>Livox Mid-360"]
    CAM["usb_cam_down<br/>下视 USB 相机"]
  end

  subgraph L1["② SLAM / 感知层"]
    FLIO["laser_mapping<br/>FAST_LIO_ROS2"]
    ATAG["apriltag_detector<br/>apriltag_ros 36h11"]
  end

  subgraph L2["③ 桥接层 scouting_bridge"]
    VPR["vision_pose_relay"]
    SOR["slam_odom_relay"]
    TFB["tf_broadcaster"]
    TPR["tag_pose_relay"]
  end

  subgraph L3["④ 规划层 ego-planner"]
    EGO["ego_planner_node<br/>FSM + 占据栅格"]
    TRAJ["traj_server"]
  end

  subgraph L4["⑤ 执行 / 任务层"]
    MIS["mission_controller"]
    CMD["cmd_bridge (C++)"]
    TAGM["tag_mission_controller"]
    FCU["MAVROS / PX4"]
  end

  LIDAR -- "/livox/lidar (CustomMsg,10Hz)<br/>/livox/imu (Imu)" --> FLIO
  CAM -- "/camera_down/image_raw/compressed<br/>+ /camera_down/camera_info" --> ATAG
  CAM -. "TF base_link→camera_down" .-> TFB

  FLIO -- "/Odometry (camera_init→body)" --> VPR
  FLIO -- "/Odometry" --> SOR
  FLIO -- "/cloud_registered(_body)" --> EGO
  FLIO -. "TF camera_init→body" .-> TFB

  ATAG -. "TF camera_down→tag36h11:id" .-> TPR
  TFB -. "TF body→base_link→camera_down" .-> TPR

  VPR -- "/mavros/vision_pose/pose" --> FCU
  SOR -- "/odom_world" --> EGO
  EGO -- "planning/bspline" --> TRAJ
  TRAJ -- "/position_cmd (100Hz)" --> CMD
  TPR -- "/tag_detection/pose + /id" --> TAGM

  CMD -- "/mavros/setpoint_raw/local" --> FCU
  MIS -- "/traj_start_trigger" --> EGO
  MIS -- "setpoint_position / set_mode / arming" --> FCU
  MIS -- "/mission/stage=complete" --> TAGM
  MIS -- "/mission/stage=complete" --> CMD
  TAGM -- "setpoint_raw + set_mode(AUTO.LAND)" --> FCU
```

---

## 3. 分环节详解

### 3.1 ① 传感器层

**Livox Mid-360**（`livox_ros_driver2`，节点名 `livox_lidar_publisher`）

| 参数 | 值 | 含义 |
|---|---|---|
| `xfer_format` | `1` | `kLivoxCustomMsg` → 发 `livox_ros_driver2/msg/CustomMsg`，**不是** PointCloud2 |
| `multi_topic` | `0` | 单话题 `/livox/lidar` |
| `publish_freq` | `10.0` | 点云 10 Hz |
| `data_src` | `0` | 真实雷达 |
| `frame_id` | `livox_frame` | ⚠️ 仅用于驱动自身话题头，FAST_LIO 会重新赋 `camera_init`/`body`/`lidar` |
| `user_config_path` | `MID360s_config.json` | 雷达 IP `192.168.1.178`，主机 IP `192.168.1.5`，点云端口 56301，IMU 端口 56401 |

输出：`/livox/lidar`（CustomMsg，10 Hz）、`/livox/imu`（sensor_msgs/Imu）

**下视 USB 相机**（`usb_cam`，节点 `usb_cam_down`，namespace `camera_down`）

| 参数 | 值 |
|---|---|
| `video_device` | `/dev/video0` |
| `pixel_format` | `raw_mjpeg` ← **本仓库打过补丁**，见下 |
| 分辨率 / 帧率 | 1280×720 @ 30 fps |
| `frame_id` | `camera_down`（与 TF 树一致） |
| `camera_info_url` | `package://scouting_bridge/config/camera_down.yaml` |

> **补丁说明**：上游 usb_cam 的 MJPEG 直通逻辑只认 ROS1 时代的字符串 `"mjpeg"`，而驱动格式校验器只接受 `"raw_mjpeg"`，
> 导致 `raw_mjpeg` 会走"把 MJPEG 缓冲当裸图"的错误分支。本仓库在 `usb_cam_node.cpp` 的 `init()` 和 `update()` 里
> 同时接受两个字符串，使真正的硬件 MJPEG 直通生效。目的是避免 1280×720 rgb8（≈2.8 MB/帧）被 NUC 上的 DDS 丢帧
> （注释记录了 2026-08-14 试飞时"收到 0 帧"的故障）。

实际发布的话题（`raw_mjpeg` 模式下）：

- `/camera_down/image_raw/compressed`（`CompressedImage`，format=`jpeg`，30 Hz）✅ 有数据
- `/camera_down/camera_info`（`CameraInfo`，30 Hz）✅ 有数据
- `/camera_down/image_raw`（`Image`）❌ **话题被 advertise 但没有任何消息**
  （`image_transport` publisher 在构造函数里无条件创建，但 MJPEG 分支只走 `take_and_send_image_mjpeg()`）

### 3.2 ② SLAM / 感知层

**FAST_LIO_ROS2**（`fast_lio`，节点 `laser_mapping`）

- 订阅：`~common.lid_topic=/livox/lidar`（`lidar_type: 1`=`AVIA` → 订阅 **CustomMsg**）、`~common.imu_topic=/livox/imu`
- 关键参数：`point_filter_num=3`、`filter_size_surf/map=0.5`、`extrinsic_est_en=true`（在线估计 IMU-LiDAR 外参）、`scan_bodyframe_pub_en=true`、`map_en=true`、`pcd_save_en=true`（⚠️ 会一直攒点云到内存，长航时会 OOM，见风险 R5）
- 输出：

| 话题 | 类型 | 坐标系 | 频率 |
|---|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | `frame_id=camera_init`，`child_frame_id=body` | ~10 Hz |
| `/cloud_registered` | `PointCloud2` | `camera_init`（世界系） | ~10 Hz |
| `/cloud_registered_body` | `PointCloud2` | `body`（机体系） | ~10 Hz |
| `/cloud_registered_lidar` | `PointCloud2` | `lidar` | ~10 Hz |
| `/Laser_map` | `PointCloud2` | `camera_init`（累积地图） | 低频 |
| `/path` | `nav_msgs/Path` | `camera_init` | ~10 Hz |
| `/cloud_effected` | `PointCloud2` | — | `effect_map_en: false`，实际不发 |
| TF `camera_init → body` | tf2 | — | ~10 Hz |

> **失效行为**：`laser_mapping` 刻意配 `respawn=False`（见 §1.2）。进程一旦退出就**不会自动重启**，`/Odometry` 与 `/cloud_registered*` 同时消失。此时 `traj_server` 会把**最后一条轨迹的终点**当作 setpoint 持续下发（其 `hover when finish traj_` 分支），所以无人机会**原地悬停**、等人工接手——不会自动恢复，也不会乱飞。`grid_map` 会在 `grid_map/odom_depth_timeout`（竞赛默认 3.0 s）后打印 `odom or depth lost!`，这是判断 SLAM 已挂的最直接信号。注意 FSM 的 `have_odom_` 是**只置位不复位**的闩锁（`ego_replan_fsm.cpp:16/286`），所以规划器自己并不会因此停车或报错。

**apriltag_ros**（节点 `apriltag_detector`）

- 订阅（经 remap）：`image_rect` → `/camera_down/image_raw`，`camera_info` → `/camera_down/camera_info`
- 由于 `image_transport: compressed`，实际订阅的是 **`/camera_down/image_raw/compressed`**
- 参数：`family: 36h11`、`size: 0.165 m`、`pose_estimation_method: pnp`、`qos_profile: sensor_data`、`max_hamming: 0`
- 输出：
  - `/detections`（`apriltag_msgs/AprilTagDetectionArray`）—— 无人订阅，仅供调试
  - **TF `camera_down → tag36h11:<id>`**：`tf.header` 直接拷贝自图像头，因此父坐标系 = `camera_down`，子坐标系名 = `<family>:<id>`
  - PnP 在 OpenCV 内部处理畸变，**所以主链路不需要 `image_proc`**

### 3.3 ③ 桥接层 scouting_bridge（胶水包核心）

包内 5 个可执行节点（`setup.py` 的 `console_scripts`），参数统一来自 `config/real_drone.yaml`。

#### vision_pose_relay —— 定位闭环的入口

```
/Odometry (camera_init→body) --[抄 pose，改写 frame_id="camera_init"]--> /mavros/vision_pose/pose
```
- 10 Hz 透传，`stamp` 原样保留（PX4 靠时间戳做延迟补偿，**千万别重新盖时间戳**）
- frame 硬编码 `camera_init`
- 这是真机唯一的 PX4 外部视觉源。对应 PX4 参数侧需要 `EKF2_EV_CTRL=7` / `EKF2_AID_MASK` 打开 vision 融合（**代码里没有自动配置，需地面站手动设**）

#### slam_odom_relay —— 控制闭环的入口

```
/Odometry --[改 frame_id: camera_init→world, child: body→base_link, 数值不变]--> /odom_world
                                                                              └-> /grid_map/odom (兜底)
```
- **刻意不发 TF**：`base_link` 的父节点必须唯一（MAVROS 的位姿只以话题形式提供），避免与 SLAM 的 TF 树冲突
- ego-planner 直接读 Odometry 里的 pose，不依赖 TF

#### tf_broadcaster —— TF 树维护者

一个节点，3 条**静态**变换（全部走 `/tf_static`，`transient_local`，开机发一次）：

| parent → child | 类型 | 来源 | 默认值 |
|---|---|---|---|
| `body → base_link` | static | 参数 | 全 0（IMU 中心 = 机体中心） |
| `base_link → livox_frame` | static | 参数 | `z=+0.10` |
| `base_link → camera_down` | static | 参数 | `z=-0.15, pitch=1.57`（≈90° 下视） |

> ⚠️ 本节点**刻意不发** `odom → base_link`。`body` 的父已经是 FAST_LIO 发的 `camera_init`，而 tf2 只允许一个 frame 有一个父节点：后写入的边会**静默覆盖**先写入的（静态边也不例外，不存在"静态优先"）。若这里再发一条高频的 `odom → base_link`，`base_link` 会被改挂到 `odom` 下，`body` 子树变成第二棵孤立的树，`body → camera_down → tag` 查找恒抛 `ConnectivityException`，`tag_pose_relay` 就永远发不出 `/tag_detection/pose`。PX4 的估计仍以 `/mavros/local_position/odom` **话题**形式提供，本工作空间没有任何地方需要它作为 TF frame。

#### tag_pose_relay —— AprilTag 世界位姿计算

- 10 Hz 定时器；订阅 `/Odometry`（BEST_EFFORT）、监听 TF
- 用 `tf_buffer._getFrameStrings()` 私有 API 扫描所有以 `tag` 开头的活动 frame，正则 `tag36h11:(\d+)` 取 ID
- 手工复合变换：`T_world→tag = T_odom(world→body，来自 /Odometry) * T_body→tag(TF 链 body→base_link→camera_down→tag)`
  （用 `scipy.spatial.transform.Rotation`，不走 tf2 的 lookup，因为 world 系没有 TF 边）
- **多标签求质心**：同一降落板上多个 tag 时，发布所有可见 tag 位置的**质心**（自然对准板中心），姿态取第一个 tag
- 输出：
  - `/tag_detection/pose`（`PoseStamped`，frame=`camera_init`）
  - `/tag_detection/id`（`Int32`；**`-1` 表示丢失**，仅在 ID 变化或丢失时发一次，不是每帧都发）
- 丢失判定：连续 `lost_threshold=3` 次（≈0.3 s）看不到任何 tag → 发 `-1`
- 陈旧检查：`odom_max_age=3.0 s`（注释说明 NUC 上 livox 时钟与系统时钟差 ~1 s，早期写死 1.0 s 导致 tag 位姿永远发不出去）

### 3.4 ④ 规划层 ego-planner

`real_drone.launch.py` 通过 `importlib` **import 并复用** `ego_planner_competition.launch.py` 的 `create_ego_planner_node()`，
然后只覆盖 map 尺寸和话题 remap。所以规划参数有两层来源：

**基础层**（`ego_planner_competition.launch.py` 的 30+ 个默认参数）+ **真机覆盖层**（`real_drone.launch.py`）：

| 参数 | competition 默认 | real_drone 覆盖 |
|---|---|---|
| `grid_map/map_size_x/y/z` | 30 / 30 / 5 | **40 / 40 / 5** |
| `grid_map/resolution` | 0.12 | **0.08** |
| `grid_map/obstacles_inflation` | 0.24 | 0.24（相同） |
| `optimization/dist0` | 0.45 | 0.45 |
| `manager/max_vel`、`optimization/max_vel`、`bspline/limit_vel` | 0.5 | **0.8** |
| `manager/max_acc`、`optimization/max_acc`、`bspline/limit_acc` | 2.0 | 2.0 |
| `fsm/flight_type` | 2 (PRESET_TARGET) | 2 |
| `fsm/planning_horizon` / `manager/planning_horizon` | 7.5 | 7.5 |
| `grid_map/local_update_range_x/y/z` | 5.5 / 5.5 / 4.5 | 继承 |
| `grid_map/max_ray_length` | 7.0 | 继承 |
| `grid_map/frame_id` | `camera_init` | 继承 |
| `manager/drone_id` | 0 | 继承（单机） |
| `prediction/obj_num` | 0 | 继承（关掉动障预测） |

**话题 remap**（`real_drone` 覆盖 competition 默认值）：

| 节点内相对名 | competition 默认 | real_drone 实际 |
|---|---|---|
| `odom_world` | `/odom_world` | `/odom_world` |
| `grid_map/odom` | `/odom_world` | `/odom_world` |
| `grid_map/cloud` | `/mid360_PointCloud2` | `/cloud_registered` |
| `grid_map/depth` | `/mid360_PointCloud2` | `/cloud_registered_body` |
| `move_base_simple/goal` | `/goal_point` | 继承（flight_type=2 时不用） |

**栅格地图的真实输入**：`plan_env/CMakeLists.txt` 里 `target_compile_definitions(plan_env PUBLIC USE_LIDAR_CLOUD)` 打开了 LiDAR 模式，
因此 `grid_map.cpp` 走 `#ifdef USE_LIDAR_CLOUD` 分支：`grid_map/depth` 按 **PointCloud2** 解析（机体系），
与 `grid_map/odom` 用 message_filters 近似时间同步，然后在 `cloudOdomCallback()` 里用 odom 的位姿把机体系点云变换到世界系做 raycast。
⚠️ 在该分支下 `grid_map/cloud`（`indep_cloud_sub_`）**根本没有被创建**，所以 `grid_map/cloud → /cloud_registered` 这条 remap 实际是空操作。

**FSM 行为**（`ego_replan_fsm.cpp`）：
- 20 ms (`execFSMCallback`) 主循环 + 50 ms (`checkCollisionCallback`) 安全检查
- 订阅 `odom_world`（remap 后 = `/odom_world`）
- `flight_type=2` 时构造阻塞等待 `/traj_start_trigger` 与 `/Odometry`，然后 `readGivenWps()` 从 `fsm/waypoint{i}_{x,y,z,yaw}` 读航点
- 航点来自 `src/ego-planner-swarm/config/competition_waypoints.yaml`（A/B/C/D 四段共 9 个航点，坐标在 `camera_init` 系）
- 输出：`planning/bspline`（`traj_utils/Bspline`）、`planning/data_display`、`grid_map/occupancy(_inflate)`

**traj_server**：
- 订阅 `planning/bspline`，10 ms 定时器 → **`/position_cmd`（`quadrotor_msgs/PositionCommand`，100 Hz）**
- 参数 `traj_server/time_forward: 1.0`

### 3.5 ⑤ 执行 / 任务层

#### cmd_bridge（C++，`/position_cmd` → `/mavros/setpoint_raw/local`）

- 订阅 `/position_cmd`（SensorDataQoS），转成 `mavros_msgs/PositionTarget`
- `coordinate_frame = FRAME_LOCAL_NED`，但**输入保持 ENU 不做手工转换** —— 源码注释专门解释：MAVROS 内部会 `transform_frame_enu_ned()`，手工再转一次就是双重转换
- `type_mask = IGNORE_YAW_RATE`：**位置 + 速度 + 加速度 + yaw 全部下发**（PX4 的 MPC 会把 offboard 速度叠加在位置环输出上，减小跟踪滞后）
- 订阅 `/mission/stage`，一旦收到 `"complete"` → `stopped_ = true`，**永久停止转发**，为 tag 降落让路

#### mission_controller（`mission_controller_competition.py`，10 Hz，6 个阶段）

| 阶段 | 动作 | 用到的话题/服务 |
|---|---|---|
| 1 OFFBOARD | 反复下发 `setpoint_position/local`（0,0,0）并调用 `set_mode(OFFBOARD)` | `/mavros/setpoint_position/local`、`/mavros/set_mode` |
| 2 Arm | `cmd/arming(true)` | `/mavros/cmd/arming` |
| 3 Takeoff | 锁定起飞点 XY，爬升到 `takeoff_height`，要求 Z 误差<0.3 且 XY 漂移<0.3，稳定 3 s | `/mavros/local_position/odom` |
| 3.5 Settle | 悬停 1 s（10 拍） | 同上 |
| 4 Trigger | 发 `/traj_start_trigger`（PoseStamped，frame=`camera_init`，位置取 **SLAM odom**） | `/traj_start_trigger` |
| 5 Handover | 等 `/position_cmd` 出现（5 s 超时 → LAND）；一旦出现立刻停止自己的 setpoint，**避免两路 setpoint 打架** | `/position_cmd` |
| 6 Monitor | 用 **SLAM `/Odometry`** 判定航点到达（`complete_dist=1.0 m`），发 `/mission/stage`，`/position_cmd` 静默>3 s 报警 | `/mission/stage` |

- **双里程计用法**：发 setpoint / 判高度用 `/mavros/local_position/odom`；判航点距离用 `/Odometry`（因为 YAML 航点就在 `camera_init` 系）
- 跑完 D 段后：`pkill -f 'ego_planner.*cmd_bridge'` **杀掉 cmd_bridge 进程**，同时发 `/mission/stage = "complete"`
- ⚠️ `takeoff_height` 实际读的是 `competition_waypoints.yaml` 的 `defaults.takeoff_height`(=1.0)，launch 里传的 `mission/takeoff_height` 等 `mission/*` 参数**没有任何代码读取**（死参数，见风险 R3）

#### tag_mission_controller（10 Hz 控制 + 20 Hz setpoint）

状态机：

```mermaid
stateDiagram-v2
    [*] --> WAIT_TRIGGER
    WAIT_TRIGGER --> HOVER_SCAN: /mission/stage=="complete"<br/>或 auto_start
    HOVER_SCAN --> TAG_DESCENT: /tag_detection/id >= 0 且 pose 存在
    HOVER_SCAN --> TIMEOUT: 超过 scan_timeout(30s)
    TIMEOUT --> RETRY_CLIMB: retry <= max_retries(3)
    TIMEOUT --> RTL: 重试耗尽
    RETRY_CLIMB --> HOVER_SCAN: 爬到 hover_z + n*1.0m
    TAG_DESCENT --> LANDED: z <= land_height(0.3m)
    LANDED --> [*]: set_mode(AUTO.LAND)
    RTL --> [*]: set_mode(AUTO.RTL)
```

降落子状态机：`DESC_LOCK_XY`（水平 PID，误差<`tag_lock_dist`(0.15 m) 连续 10 拍）→ `DESC_LOWER`（每步降 `descend_step`(0.5 m)）→ `DESC_FINAL`。

- **水平控制用 SLAM `/Odometry`**（与 tag 位姿同系），速度由 PID(`kp=0.5, ki=0.02, kd=0.15`) 算出并限幅 `max_horizontal_vel=1.0`，通过 `PositionTarget`（XY 速度 + Z 位置 + yaw）下发
- **垂直/偏航用 MAVROS odom**
- `WAIT_TRIGGER` 期间**完全不发 setpoint**（此时是 cmd_bridge + competition controller 在控）
- 触发瞬间**同步补发一次** setpoint，填补 competition controller 停发造成的空档（PX4 OFFBOARD 需要连续 setpoint，否则 ~0.5 s 后触发 failsafe）
- 触发后它自己也往 `/mission/stage` 发状态名（`HOVER_SCAN`/`TAG_DESCENT`/…/`RTL`），与 competition controller 共用同一话题

#### 视觉辅助：odom_visualization

订阅 `odom`（remap 到 `/Odometry`）与 `/position_cmd`，发布 `/robot`(MESH marker)、`/pose`、`/path`、`/trajectory` 等，仅供 RViz。

---

## 4. 全量话题/服务清单

图例：**P**=发布者，**S**=订阅者/调用方

| 话题 / 服务 | 类型 | P | S | 说明 |
|---|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | livox_lidar_publisher | laser_mapping | 10 Hz，`xfer_format=1` |
| `/livox/imu` | `sensor_msgs/Imu` | livox_lidar_publisher | laser_mapping | |
| `/Odometry` | `nav_msgs/Odometry` | laser_mapping | vision_pose_relay, slam_odom_relay, tag_pose_relay, tag_mission_controller, mission_controller, odom_visualization | `camera_init→body`，10 Hz，**系统里被订阅最多的话题** |
| `/cloud_registered` | `PointCloud2` | laser_mapping | (ego_planner 的 remap 目标，实际未被订阅) | 世界系点云 |
| `/cloud_registered_body` | `PointCloud2` | laser_mapping | ego_planner（`grid_map/depth`） | 机体系点云，栅格地图真正吃的数据 |
| `/cloud_registered_lidar` / `/Laser_map` / `/path` | PointCloud2 / Path | laser_mapping | RViz | |
| TF `camera_init→body` | tf2 | laser_mapping | tf 树 | |
| `/camera_down/image_raw/compressed` | `CompressedImage` | usb_cam_down | apriltag_detector | 30 Hz，jpeg |
| `/camera_down/camera_info` | `CameraInfo` | usb_cam_down | apriltag_detector | 30 Hz |
| TF `camera_down→tag36h11:<id>` | tf2 | apriltag_detector | tag_pose_relay | 每个 tag 一条 |
| `/detections` | `apriltag_msgs/AprilTagDetectionArray` | apriltag_detector | — | 仅调试 |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | vision_pose_relay | mavros → PX4 EKF2 | 定位闭环 |
| `/odom_world` | `nav_msgs/Odometry` (frame `world`) | slam_odom_relay | ego_planner (`odom_world`, `grid_map/odom`) | 控制闭环 |
| `/grid_map/odom` | `nav_msgs/Odometry` | slam_odom_relay | — | 兜底，被 remap 后无订阅者 |
| TF `body→base_link` / `base_link→livox_frame` / `base_link→camera_down` | tf2 static | tf_broadcaster | tf 树 | 开机发一次（`/tf_static`） |
| `/tag_detection/pose` | `PoseStamped` (`camera_init`) | tag_pose_relay | tag_mission_controller | 10 Hz |
| `/tag_detection/id` | `std_msgs/Int32` | tag_pose_relay | tag_mission_controller | `-1`=丢失，仅变化时发 |
| `planning/bspline` | `traj_utils/Bspline` | ego_planner_node | traj_server | 重规划时 |
| `planning/data_display` | `traj_utils/DataDisp` | ego_planner_node | RViz | |
| `grid_map/occupancy` / `_inflate` | `PointCloud2` | ego_planner_node | RViz | ~9 Hz |
| `/position_cmd` | `quadrotor_msgs/PositionCommand` | traj_server | cmd_bridge, mission_controller(监听) | **100 Hz** |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | cmd_bridge, tag_mission_controller | mavros → PX4 | ENU 输入，MAVROS 内部转 NED |
| `/mavros/setpoint_position/local` | `PoseStamped` | mission_controller, tag_mission_controller | mavros → PX4 | 起飞 / 悬停 |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | mavros | mission_controller, tag_mission_controller | PX4 EKF 输出（**仅话题**，不再进 TF） |
| `/mavros/state` | `mavros_msgs/State` | mavros | mission_controller, tag_mission_controller | armed / mode / connected |
| `/mavros/set_mode` (srv) | `mavros_msgs/SetMode` | mission_controller, tag_mission_controller | mavros | OFFBOARD / AUTO.LAND / AUTO.RTL |
| `/mavros/cmd/arming` (srv) | `mavros_msgs/CommandBool` | mission_controller, tag_mission_controller | mavros | |
| `/traj_start_trigger` | `geometry_msgs/PoseStamped` | mission_controller | ego_planner_node | 一次性 |
| `/mission/stage` | `std_msgs/String` | mission_controller, tag_mission_controller | cmd_bridge, tag_mission_controller | 任务交接总线 |
| `/robot` 等 | `visualization_msgs/Marker` | odom_visualization | RViz | |

---

## 5. TF 树与坐标系

```mermaid
graph TD
  CI["camera_init<br/>(SLAM 世界原点)"]
  BD["body<br/>(LiDAR IMU 中心)"]
  BL["base_link<br/>(机体中心)"]
  LV["livox_frame"]
  CD["camera_down"]
  TG["tag36h11:&lt;id&gt;"]

  CI -->|"dynamic ~10Hz<br/>FAST_LIO"| BD
  BD -->|"static 全 0<br/>tf_broadcaster"| BL
  BL -->|"static z=+0.10"| LV
  BL -->|"static z=-0.15, pitch=1.57"| CD
  CD -->|"dynamic 每帧<br/>apriltag_ros PnP"| TG
```

> 树里**没有** `odom`：`base_link` 的父是 `body`。PX4 的 `odom` 只作为**话题**坐标系存在，不与 SLAM 树做 TF 连接。

**坐标系约定（非常重要）：**

1. `camera_init` = `world`（SLAM 侧，数值等价）。但它与 **PX4 的本地系**（本仓库旧代码里称之为 `odom`；MAVROS `px4_config.yaml` 里 `local_position` 实际用的是 `map`）只是"**应该**重合"——靠上电时两套系统同时归零、且 EKF2 融合了 vision 的**位置与 yaw** 来隐式维持，系统里**没有任何代码去对齐或校验**。注意 `camera_init` 的 yaw 来自 IMU 初始姿态（本质上是任意的），PX4 的 yaw 来自 EKF 的磁/视觉得融合结果，两者是否一致是飞行正确性的前提。详见 **R10 / §8.1**。
2. 所有话题坐标都是 **ENU**（x 东、y 北、z 上）。`cmd_bridge.cpp` 和 `tag_mission_controller` 都明确注释：`FRAME_LOCAL_NED` 常量名下 MAVROS 期望 ENU 输入，**不要手工转 NED**。
3. `competition_waypoints.yaml`、`/tag_detection/pose`、`slam_odom_relay` 输出都在 `camera_init` 系。
4. `grid_map/frame_id = camera_init`，与外部点云/里程计一致。
5. `tf_broadcaster` 的 `camera_down` 用 **pitch=+1.57** 表示下视——符号是否与实际安装一致，需要用 tag 的 z 值正负来验证（风险 R2）。

---

## 6. 任务时序（从起飞到降落）

```mermaid
sequenceDiagram
  participant PX4 as PX4/MAVROS
  participant FLIO as FAST_LIO
  participant EGO as ego_planner
  participant MC as mission_controller
  participant CB as cmd_bridge
  participant TMC as tag_mission_controller

  Note over PX4,FLIO: 上电，SLAM 与 PX4 同时归零
  FLIO->>PX4: /Odometry → /mavros/vision_pose/pose (EKF2 融合)
  MC->>PX4: set_mode(OFFBOARD) + arming
  MC->>PX4: setpoint_position/local 爬升到 1.0m
  MC->>EGO: /traj_start_trigger (camera_init)
  EGO->>EGO: readGivenWps() 载入 A/B/C/D 共 9 个航点
  EGO->>CB: planning/bspline → traj_server → /position_cmd
  Note over MC,CB: MC 检测到 /position_cmd 后停止 setpoint（交接完成）
  loop 每个航点
    EGO->>EGO: 重规划
    CB->>PX4: /mavros/setpoint_raw/local (pos+vel+acc+yaw)
    MC->>MC: 用 /Odometry 判距，发 /mission/stage=A|B|C|D
  end
  MC->>CB: /mission/stage = "complete" (+ pkill cmd_bridge)
  CB->>CB: stopped_=true，停止转发
  MC->>TMC: /mission/stage = "complete"
  TMC->>PX4: setpoint_position/local 悬停（补发一次，避免 OFFBOARD failsafe）
  loop 扫描
    TMC->>TMC: 等 /tag_detection/id >= 0
  end
  TMC->>PX4: setpoint_raw/local（XY 速度 PID + Z 位置）
  TMC->>PX4: set_mode(AUTO.LAND)
```

---

## 7. 参数生效链路速查

| 行为 | 改哪里 |
|---|---|
| TF 外参（LiDAR/相机安装位置） | `src/scouting_bridge/config/real_drone.yaml` → `tf_broadcaster.*` |
| 相机内参 / 畸变 | `src/scouting_bridge/config/camera_down.yaml` |
| AprilTag 家族 / 尺寸 / 检测调参 | `src/scouting_bridge/config/apriltag_config.yaml` |
| 相机设备号 / 分辨率 / 帧率 / 像素格式 | `src/scouting_bridge/config/usb_cam_down.yaml` |
| 航点、起飞高度、完成半径 | `src/ego-planner-swarm/config/competition_waypoints.yaml` |
| 规划器基础参数 | `ego_planner_competition.launch.py` 的 `params` |
| 真机规划器覆盖（地图尺寸/速度/分辨率） | `real_drone.launch.py` 的 `param_overrides` |
| SLAM 参数（滤波、外参、体素） | `src/FAST_LIO_ROS2/config/mid360.yaml` |
| 雷达 IP / 端口 | `src/livox_ros_driver2/config/MID360s_config.json` |
| 飞控连接 | 飞控侧 MAVROS 启动配置的 `fcu_url`（不在本仓库内） |
| 降落 PID / 超时 / 重试 | `real_drone.launch.py` 的 `tag_mission_node` 参数块 |

---

## 8. 风险点与待验证项

> 以下都是读代码得到的**事实性观察**，不是猜测；每条都给出验证方法。

| # | 风险 | 依据 | 建议 |
|---|---|---|---|
| **R1** | MAVROS 的启动配置**不在本仓库内**，有两项必须自行确认：① `fcu_url` 要指向真机链路（SITL 回环 `udp://127.0.0.1:14552@127.0.0.1:14552` 在真机上收不到任何数据）；② `tf.send` 必须为 `false`——否则 MAVROS 会再发一条 `map→base_link`，与 `tf_broadcaster` 的 `body→base_link` 争夺 `base_link` 的父节点（tf2 后写覆盖先写，`body` 子树会变成孤立树，`tag_pose_relay` 的 TF 查找恒抛 `ConnectivityException`）。`px4_config.yaml:56,75` 里已是 `false`，但要确认当前启动方式确实加载了这份配置 | 飞控侧 MAVROS 启动配置 | 起飞前两项各一条命令：`ros2 topic echo /mavros/state` 应 `connected=true`；`ros2 run tf2_ros tf2_echo body camera_down` 必须出数，而不是报 `two or more unconnected trees` |
| **R2** | `camera_down` 的 `pitch=+1.57` 方向未经代码校验 | `real_drone.yaml:36` | 看 `/tag_detection/pose` 的 z 相对机体是否合理（下视相机观测到的地面 tag 应在机体下方） |
| **R3** | `mission_controller` 的 launch 参数 `mission/*`（takeoff_height、navigation_height、max_flight_height…）**全是死参数**，节点只从 `competition_waypoints.yaml` 的 `defaults` 读 `takeoff_height`/`complete_dist` | `mission_controller_competition.py:294,447`；`real_drone.launch.py:283-290` | 要改起飞高度请改 YAML，或补上参数读取 |
| **R4** | `grid_map/cloud → /cloud_registered` remap 在 `USE_LIDAR_CLOUD` 模式下**无效**（该订阅未创建），真正吃的是 `grid_map/depth → /cloud_registered_body` | `grid_map.cpp:140-152`、`plan_env/CMakeLists.txt:71` | 无害，但别误以为世界系点云在被使用 |
| **R5** | FAST_LIO `pcd_save_en: true` + `interval: -1` 会把整段飞行点云全存内存 | `mid360.yaml:47-49` | 长航时前改为 `false` |
| **R6** | `mission_controller` 在任务完成时 `pkill -f 'ego_planner.*cmd_bridge'` 强杀进程，同时 `cmd_bridge` 也会因 `/mission/stage=complete` 自行停止 —— 双保险，但如果节点名/可执行路径变化，pkill 会失效 | `mission_controller_competition.py:495-497`、`cmd_bridge.cpp:50-55` | 依赖 cmd_bridge 自身的 `stopped_` 逻辑即可，pkill 是冗余保障 |
| **R7** | `tag_pose_relay` 使用 `tf_buffer._getFrameStrings()` 私有 API | `tag_pose_relay.py:209` | 升级 tf2 时可能失效 |
| **R8** | `tag_pose_relay` 的 `odom_max_age=3.0s` 是对**雷达时钟与主机时钟未对齐**的变通：`/Odometry` 的 stamp 与主机墙钟相差一个常数（原代码注释记录约 1s） | `tag_pose_relay.py:55-60` | 用 `ros2 topic delay /Odometry` 确认偏差；根治需在雷达侧开启时间同步（暂不展开）。**注意 `mid360.yaml` 的 `time_sync_en` 不是这个开关，勿误改**；确认对齐后再把该值调回 ~0.5 s |
| **R9** | `/mission/stage` 被 `mission_controller` 与 `tag_mission_controller` **双发布**，且触发词 `"complete"` 同时作为 cmd_bridge 的停止信号和 tag 任务的启动信号（一话题三语义） | 两个节点的 `_stage_pub` | 重放/重启时注意：任意一方先发 `complete` 都会触发交接 |
| **R10** | **规划系 ≠ 控制系**：ego-planner 吃 `camera_init` 系的 `/odom_world` 与点云，输出的 `/position_cmd` 却由 PX4 在**它自己的本地系**里实现。只有 `Δ = T(PX4系 ← camera_init) = I` 时二者才等价，而这个等式仅靠"`vision_pose_relay` 把 FAST_LIO 喂进 EKF2"隐式维持，**代码里没有任何一处强制或校验**。`Δψ` 会把整条航线在真实世界中旋转（15 m 场地 10° ≈ 远端 2.6 m），且失效**静默**——`mission_controller` 用 SLAM 判到达，会照常报"航点已到" | `slam_odom_relay.py:62`、`cmd_bridge.cpp:73`（`frame_id` 写死 `"map"`）、`vision_pose_relay.py:44-46` | 起飞前在地面静止时对比两路位姿，确认 `\|Δψ\| < 2°` 且 `\|Δt\| < 0.2 m`；展开见 §8.1 |

### 8.1 R10 展开：规划系 / 控制系的对齐假设

**误差模型.** 设 `Δ = T(PX4 本地系 ← camera_init)`。ego-planner 在 `camera_init` 里算出目标 X，`cmd_bridge` 把数值原样交给 MAVROS（还把它标成 `frame_id="map"`），PX4 **只认数值**、在自己的本地系里调节。所以真实落点是 `T_Δ(X)`：

| Δ 的分量 | 后果 |
|---|---|
| 平移 `t_Δ` | 整条航线平移 `t_Δ` |
| 航向 `Δψ` | 整条航线在真实世界里绕原点旋转（15 m 场地、10° ≈ 远端 2.6 m） |
| 高度 `Δz` | C/D 段航点 z=0.30 m，0.3 m 偏差即撞地量级 |

**为什么现在能跑.** `vision_pose_relay` 把 FAST_LIO 位姿喂进 EKF2，而 MAVROS/PX4 **不读 `frame_id` 字符串、只取数值当本地系的量**，于是 EKF2 的本地系会被"拉"到与 `camera_init` 重合。但这需要三个条件同时成立，且**全部在本仓库之外**：

1. 两系原点都在起飞前静止时确定；
2. EKF2 融合 vision 的 **position 与 yaw**（`EKF2_EV_CTRL`）——若 yaw 位没开，`camera_init` 的 yaw（IMU 初始姿态，本质任意）与 PX4 的 yaw（磁/视觉得融合结果）可以差几十度；
3. 高度基准一致（`EKF2_HGT_MODE`）——若用气压计而 SLAM 的 z 缓慢漂移，`Δz` 会随时间增长。

**失效场景**

- **SLAM 进程退出**：不会自动重启，无人机会原地悬停等人工接手——具体行为与判断信号见 §3.2。此时不再产生新的规划，也不会引入额外的坐标系错误。
- **场馆磁干扰**使 EKF2 偏向磁力计而非 vision yaw → `Δψ` 可达几十度；飞机照飞、照报"到达"。

**建议的验证（起飞前在地面静止，5 分钟）**

```bash
ros2 topic echo /Odometry                   --field pose.pose --once
ros2 topic echo /mavros/local_position/odom --field pose.pose --once
# 各自把四元数转成 yaw，再连同位置相减，得到 Δψ 与 Δt
```

**两个干净的终态**

| | 终态 X（现状） | 终态 Y（全面迁到 PX4 系） |
|---|---|---|
| 规划 / 建图世界系 | `camera_init` | PX4 本地系 |
| 控制系 | PX4 本地系 | PX4 本地系 |
| `Δ` 的影响 | 依赖假设 `Δ≈0`，失效即静默偏航 | **从构造上无关**（三者同系） |
| 要改什么 | 无（建议补 Δ 监控 + 起飞前自检） | `odom_world`、`grid_map/odom`、`grid_map/frame_id`、`competition_waypoints.yaml`、`tag_pose_relay.world_frame`，**必须同时**换 |
| 代价 | — | 建图引入 EKF 延迟与 EKF 的 z 语义；需重新标定航点并重飞验证 |

> ⚠️ **只换一半才会真正错位**：若只把 `odom_world` 换成 EKF 里程计而 `grid_map/odom` 仍用 SLAM，则规划器状态在 PX4 系、地图却在 `camera_init`。两个都换时，`cloudOdomCallback()` 算的是 `p_map = T(base_link in map) · p_body`，在本地系里是**自洽**的。

---

## 9. 常用调试命令

```bash
# 一键看谁在发 /Odometry、频率多少
ros2 topic hz /Odometry
ros2 topic info /Odometry --verbose

# TF 树体检（每个 frame 只能有一个父节点；body→camera_down 必须能查到）
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo body camera_down
ros2 run tf2_ros tf2_echo body tag36h11:0

# 定位闭环是否通到 PX4
ros2 topic hz /mavros/vision_pose/pose
ros2 topic echo /mavros/local_position/odom --once

# 起飞前自检（R10）：规划系 camera_init 与控制系 PX4 是否对齐（地面静止时做）
ros2 topic echo /Odometry                   --field pose.pose --once
ros2 topic echo /mavros/local_position/odom --field pose.pose --once
# 把两组四元数各自转成 yaw，连位置一起相减 → Δψ 与 Δt
# 判据：|Δψ| < 2° 且 |Δt| < 0.2 m。超出则整条航线会被旋转/平移，详见 §8.1

# 控制闭环
ros2 topic hz /position_cmd
ros2 topic echo /mavros/setpoint_raw/local --once
ros2 topic echo /mavros/state

# 任务交接
ros2 topic echo /mission/stage
ros2 topic echo /tag_detection/id
ros2 topic hz /camera_down/image_raw/compressed

# 栅格地图可视化（RViz 里加 grid_map/occupancy_inflate）
ros2 topic hz /grid_map/occupancy_inflate
```

---

## 10. 包职责一览

| 包 | 角色 | 是否本仓库自研 |
|---|---|---|
| `scouting_bridge` | **胶水包**：SLAM↔PX4↔ego-planner 的话题/坐标系/任务交接，含 TF 维护与 AprilTag 降落任务 | ✅ 自研 |
| `ego-planner-swarm` | 规划器；本仓库在 `plan_env` 开了 `USE_LIDAR_CLOUD`、新增 `cmd_bridge.cpp`、竞赛 launch 与 mission controller | ⚠️ 上游 + 大量本地改动 |
| `FAST_LIO_ROS2` | LiDAR-惯性里程计与建图 | 上游 |
| `livox_ros_driver2` | Mid-360 驱动 | 上游 |
| `apriltag_ros` | AprilTag 检测 + PnP 位姿（输出 TF） | 上游 |
| `usb_cam` | 下视相机驱动 | ⚠️ 上游 + MJPEG 直通补丁 |

---