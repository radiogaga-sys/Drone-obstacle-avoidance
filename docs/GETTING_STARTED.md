# UAV 避障系统 · 上手指南

> 面向**第一次接触本仓库**的人。目标：看完能自己改对一个参数，并知道怎么验证它生效。
> 深入的数据流/话题清单/风险分析见 [`DATA_FLOW.md`](DATA_FLOW.md)；交互式数据流图见 [`UAV_DATAFLOW.html`](UAV_DATAFLOW.html)。
> 平台：ROS 2 Humble + PX4/MAVROS + Livox Mid-360 + 下视 USB 相机。

---

## 0. 先建立一个心智模型（1 分钟）

这套系统做的事：**用激光雷达做定位、用 ego-planner 做避障规划、用 MAVROS 把指令交给 PX4 飞，最后用下视相机识别 AprilTag 精准降落。**

理解它只需要抓住 **两条闭环**：

| 闭环 | 作用 | 一句话链路 |
|---|---|---|
| **A. 定位闭环** | 让 PX4 知道"我在哪" | FAST_LIO 的 `/Odometry` → `vision_pose_relay` → `/mavros/vision_pose/pose` → **PX4 EKF2 融合** → `/mavros/local_position/odom` |
| **B. 控制闭环** | 让无人机按规划飞 | `/Odometry` → `slam_odom_relay` → `/odom_world` → **ego-planner** → `traj_server` → `/position_cmd` → `cmd_bridge` → `/mavros/setpoint_raw/local` → PX4 |

七个包各司其职：

```
livox_ros_driver2 ─┐
                   ├─► FAST_LIO_ROS2 ──► scouting_bridge ──► ego-planner-swarm ──► PX4
usb_cam ─► apriltag_ros ─┘   （胶水包）      （规划器）
```

> ⚠️ **全系统最重要的一个约定**：SLAM 的 `camera_init` 与 PX4 的本地系（MAVROS 里叫 `map`）被当作**同一个原点**，代码里没有任何校验。这是本系统最大的隐性风险，详见 [`DATA_FLOW.md` §8 R10](DATA_FLOW.md)。

---

## 1. 环境与编译

### 1.1 依赖

| 依赖 | 谁需要 |
|---|---|
| ROS 2 Humble（desktop） | 全部 |
| `mavros` + `mavros_msgs` | 飞控通信 |
| PCL / Eigen | FAST_LIO、ego-planner |
| **Livox SDK2**（`/usr/local/include/livox_lidar_*.h`） | livox_ros_driver2 |
| `apriltag` 库 | apriltag_ros |
| `python3-scipy` | `tag_pose_relay`（做旋转复合） |
| ffmpeg / v4l-utils | usb_cam |

各包的确切依赖看它自己的 `package.xml`。

### 1.2 编译

```bash
cd <工作空间根目录>          # 即含 src/ 的那一层
colcon build --symlink-install
source install/setup.bash
```

> **建议始终加 `--symlink-install`**：它会把你改的 `.py` / `.yaml` / `.launch.py` 直接软链到 `install/`，改完不用重编。
> ⚠️ **例外**：**新增**文件（多加一个 `config/xxx.yaml` 或 `launch/xxx.launch.py`）仍然必须重新 `colcon build`——因为 `setup.py` 里的 `glob("config/*.yaml")` 是在**构建时**求值的。

### 1.3 目录结构

```
src/
├── scouting_bridge/        ← 胶水包（本仓库自研，你最常改的就是它）
│   ├── config/             ← 所有可调参数都在这里
│   ├── launch/             ← 启动文件
│   └── scouting_bridge/    ← 5 个胶水节点
├── ego-planner-swarm/      ← 规划器（上游 + 大量本地改动）
│   └── config/competition_waypoints.yaml   ← 航点
├── FAST_LIO_ROS2/          ← LiDAR-惯性里程计（上游）
│   └── config/mid360.yaml
├── livox_ros_driver2/      ← 雷达驱动（上游）
├── apriltag_ros/           ← AprilTag 检测（上游）
└── usb_cam/                ← 相机驱动（上游 + MJPEG 补丁）
docs/                       ← 本文档 + 数据流详解
```

---

## 2. 硬件连接与启动

### 2.1 硬件连接

| 设备 | 接到 | 备注 |
|---|---|---|
| Livox Mid-360 | NUC 的**有线网口**（RJ45） | 直连或同一交换机；**不要接 PoE** |
| PX4 飞控 | NUC 的串口或 USB | 具体链路决定 MAVROS 的 `fcu_url` |
| 下视 USB 相机 | NUC 的 USB 口 | 默认 `/dev/video0` |

### 2.2 启动顺序（两步）

```bash
# 终端 1：先起 MAVROS —— 本仓库不提供它的启动文件，按你们既有的方式启动
#         必须确认 fcu_url 指向真机链路，不能用 SITL 回环地址

# 终端 2：一键起所有（SLAM + 桥接 + 相机 + 规划 + 任务）
ros2 launch scouting_bridge real_drone.launch.py
```

常用启动参数：

```bash
ros2 launch scouting_bridge real_drone.launch.py enable_mission:=false    # 不跑比赛任务
ros2 launch scouting_bridge real_drone.launch.py enable_apriltag:=false   # 关掉相机链路
ros2 launch scouting_bridge real_drone.launch.py rviz:=true               # 台架调试开 RViz
```

其他两个启动文件：
- `real_drone_no_tag.launch.py` —— 去掉 AprilTag 链路的精简版
- `apriltag_pipeline.launch.py` —— **只起相机链路**，用来单独调试检测器

### 2.3 怎么确认"起来了"

按顺序敲，每一步都要有输出才继续：

```bash
ros2 topic hz /livox/lidar              # 雷达出点，应 ~10 Hz
ros2 topic hz /Odometry                 # SLAM 出里程计，应 ~10 Hz
ros2 topic echo /mavros/state --once    # 应 connected: true
ros2 topic hz /position_cmd             # 规划器出指令，应 ~100 Hz
```

---

## 3. 【最常用】"我要改 X，去哪改"

先看这张总表，细节在 §4：

| 我要改… | 改这个文件 |
|---|---|
| **传感器之间的外参**（LiDAR/相机装在哪） | ⚠️ **有两个地方，见 §4.1** |
| **雷达 IP / 网络** | `src/livox_ros_driver2/config/MID360s_config.json` |
| 相机设备号 / 分辨率 / 帧率 | `src/scouting_bridge/config/usb_cam_down.yaml` |
| 相机内参 / 畸变 | `src/scouting_bridge/config/camera_down.yaml` |
| AprilTag 家族 / 标签边长 | `src/scouting_bridge/config/apriltag_config.yaml` |
| 航点 / 起飞高度 / 到达判定半径 | `src/ego-planner-swarm/config/competition_waypoints.yaml` |
| 规划器（地图尺寸、速度、分辨率、膨胀） | `src/scouting_bridge/launch/real_drone.launch.py` 的 `param_overrides` |
| SLAM 滤波 / 体素 / 外参 / 存图 | `src/FAST_LIO_ROS2/config/mid360.yaml` |
| 降落 PID / 超时 / 重试 | `src/scouting_bridge/launch/real_drone.launch.py` 的 `tag_mission_node` |
| 飞控连接（`fcu_url`）、MAVROS 行为 | **不在本仓库**，在飞控侧的 MAVROS 启动配置里 |

> 改完 `.yaml` / `.launch.py` 后：用了 `--symlink-install` 的话直接重启 launch 即可；否则要 `colcon build --packages-select <包名>`。

---

## 4. 逐项展开

### 4.1 传感器外参 ⚠️ 有两个，千万别搞混

这是新手最容易错的地方：**"外参"在本仓库分成两套，改一套不会影响另一套。**

| | **外参 A：IMU ↔ LiDAR** | **外参 B：各传感器 ↔ 机体中心** |
|---|---|---|
| 文件 | `src/FAST_LIO_ROS2/config/mid360.yaml` | `src/scouting_bridge/config/real_drone.yaml` |
| 字段 | `mapping.extrinsic_T`（米）、`mapping.extrinsic_R`（3×3 旋转矩阵） | `tf_broadcaster.body_*` / `mid360_*` / `camera_down_*` |
| 物理含义 | LiDAR 在 **IMU 机体坐标系**下的位姿 | 各传感器相对**机体中心**的静态变换 |
| 谁消费 | FAST_LIO 内部（运动畸变补偿、建图） | TF 树 → `tag_pose_relay` 算 tag 世界坐标 |
| 单位 | 米 / 旋转矩阵 | 米 / **弧度**（`1.57` = 90°，不是度！） |

```yaml
# 外参 A：src/FAST_LIO_ROS2/config/mid360.yaml
mapping:
    extrinsic_est_en:  true              # ← true 时会在线估计，初值只是起点
    extrinsic_T: [ -0.011, -0.02329, 0.04412 ]
    extrinsic_R: [ 1., 0., 0.,
                   0., 1., 0.,
                   0., 0., 1.]
```

```yaml
# 外参 B：src/scouting_bridge/config/real_drone.yaml
tf_broadcaster:
  ros__parameters:
    # body(IMU 中心) -> base_link(机体中心)
    body_x: 0.0
    body_y: 0.0
    body_z: 0.0
    body_roll: 0.0
    body_pitch: 0.0
    body_yaw: 0.0

    # base_link -> livox_frame（雷达装在机体中心的哪）
    mid360_x: 0.0
    mid360_y: 0.0
    mid360_z: 0.1

    # base_link -> camera_down（下视相机）
    camera_down_x: 0.0
    camera_down_y: 0.0
    camera_down_z: -0.15
    camera_down_pitch: 1.57        # ← 弧度！≈90° 表示镜头朝下
```

**规则**：只要你**动了雷达或相机的物理位置**，A 和 B **都要改**。

- 挪了雷达 → 改 A 的 `extrinsic_T`，同时改 B 的 `mid360_*`
- 挪了相机 → 只改 B 的 `camera_down_*`（相机不参与 SLAM，所以 A 不动）
- 调完怎么验：`ros2 run tf2_ros tf2_echo body camera_down` 应出数；再对着地面放一个 tag，看 `/tag_detection/pose` 的 z 是否在机体下方

> ⚠️ 外参 B 的 `body_*` 默认全 0，意思是"IMU 中心 = 机体中心"。如果实际不重合而你没填，**tag 降落会有固定偏差**。

### 4.2 雷达 IP 与网络

**文件：`src/livox_ros_driver2/config/MID360s_config.json`**

```json
{
  "lidar_summary_info": { "lidar_type": 8 },      // 8 = Livox 私有协议，别改
  "Mid360s": {
    "lidar_net_info": {                            // 雷达侧端口，别改
      "cmd_data_port": 56100, "push_msg_port": 56200,
      "point_data_port": 56300, "imu_data_port": 56400, "log_data_port": 56500
    },
    "host_net_info": [{
      "host_ip": "192.168.1.5",                    // ★ 主机 IP —— 你要改的
      "cmd_data_port": 56101, "push_msg_port": 56201,
      "point_data_port": 56301, "imu_data_port": 56401, "log_data_port": 56501
    }]
  },
  "lidar_configs": [{
    "ip": "192.168.1.178",                         // ★ 雷达 IP —— 你要改的
    "pcl_data_type": 1,                            // 1 = 高精度笛卡尔坐标（mm），别改
    "pattern_mode": 0,                             // 0 = 非重复扫描，别改
    "extrinsic_parameter": { "roll":0,"pitch":0,"yaw":0,"x":0,"y":0,"z":0 }  // 见下方警告
  }]
}
```

**两个 IP 都要对**：

- `lidar_configs[0].ip` = **雷达自己的 IP**。Livox 的默认规则是 `192.168.1.1xx`（xx = 序列号后两位），**以设备标签为准**。当前配置里是 `192.168.1.178`。
- `host_net_info[0].host_ip` = **主机网卡的 IP**。你的 NUC 那个网口必须**真的配上这个地址**：

```bash
# 检查 NUC 网口地址（示例网卡名 enp2s0，用 ip a 看实际名字）
ip a show enp2s0
# 如果不是 192.168.1.5，配上去（临时）
sudo ip addr add 192.168.1.5/24 dev enp2s0

# 验证能通
ping 192.168.1.178
```

> ⚠️ `extrinsic_parameter` 这里是**下发到雷达固件**的外参，与 §4.1 的两套外参都无关，保持全 0 即可。改错了会让点云整体歪掉。
> ⚠️ 改完这个 JSON 后**必须重新 `colcon build --packages-select livox_ros_driver2`**（它是构建时拷贝的）。

**验证**：`ros2 topic hz /livox/lidar` 有 ~10 Hz；`ros2 topic hz /livox/imu` 有数据。

### 4.3 相机

| 文件 | 改什么 |
|---|---|
| `src/scouting_bridge/config/usb_cam_down.yaml` | `video_device`（`/dev/video0`）、`image_width/height`、`framerate`、`pixel_format`（保持 `raw_mjpeg`） |
| `src/scouting_bridge/config/camera_down.yaml` | 相机内参 `camera_matrix`、畸变 `distortion_coefficients`（换相机或换分辨率后必须重标定） |

> ⚠️ **别把 `pixel_format` 改成 `mjpeg2rgb`**。本仓库给 usb_cam 打了补丁，用 `raw_mjpeg` 走**硬件 MJPEG 直通**（发布 `/camera_down/image_raw/compressed`）。改成解码模式会产生 2.8 MB/帧的裸图，在 NUC 上会被 DDS 丢光（历史上真的发生过，收到 0 帧）。
> ⚠️ 改分辨率**不会**自动缩放内参。非标定分辨率下 tag 位姿不可信。

**验证**：`ros2 topic hz /camera_down/image_raw/compressed` 应 ~30 Hz。

### 4.4 AprilTag

**文件：`src/scouting_bridge/config/apriltag_config.yaml`**

| 参数 | 说明 |
|---|---|
| `family` | 标签家族，默认 `36h11`。**必须和你实际打印的标签一致** |
| `size` | 标签**黑框边长**（米），默认 `0.165`。**量错会让距离整体缩放错误** |
| `image_transport` | 保持 `compressed`（配合相机的 MJPEG 直通） |
| `pose_estimation_method` | `pnp`（准确）或 `homography`（快） |
| `detector.decimate` | 1.0 = 全分辨率，调大更快但识别距离更近 |

### 4.5 航点与任务

**文件：`src/ego-planner-swarm/config/competition_waypoints.yaml`**

```yaml
stages:
  A:
    waypoints:
      - [x, y, z]        # 坐标系：camera_init（= 无人机上电位置为原点）
      - [x, y, z]
  B: { ... }
  C: { ... }
  D: { ... }

defaults:
  takeoff_height: 1.0    # 起飞高度（米）
  complete_dist: 1.0     # 航点到达判定半径（米）
  speed: 0.5             # 巡航速度，要和 launch 里的 max_vel 保持一致
```

> ⚠️ 改**起飞高度**请改这里，不要改 launch 里的 `mission/takeoff_height`——那几个 `mission/*` 参数**是死参数，没有任何代码读取**。
> ⚠️ 航点是**上电时刻**的坐标系，不是地理坐标。换起飞位置 = 换一套航点。

### 4.6 规划器参数

`real_drone.launch.py` 里用 `importlib` **复用**了 `ego_planner_competition.launch.py` 的默认参数，然后只覆盖这几项：

```python
# src/scouting_bridge/launch/real_drone.launch.py
param_overrides={
    "grid_map/map_size_x": 40.0, "grid_map/map_size_y": 40.0, "grid_map/map_size_z": 5.0,
    "grid_map/resolution": 0.08,          # 越小越精细、越吃 CPU
    "grid_map/obstacles_inflation": 0.24, # 障碍膨胀（安全半径）
    "optimization/dist0": 0.45,           # 离障碍的最小距离
    "manager/max_vel": 0.8, "manager/max_acc": 2.0,      # ← 三个地方要同步改！
    "optimization/max_vel": 0.8, "optimization/max_acc": 2.0,
    "bspline/limit_vel": 0.8, "bspline/limit_acc": 2.0,
}
```

> ⚠️ 改速度/加速度时，**`manager` / `optimization` / `bspline` 三组必须一致**，只改一个会出现规划器内部矛盾。
> 其余参数在 `ego_planner_competition.launch.py` 里。

### 4.7 SLAM 参数

**文件：`src/FAST_LIO_ROS2/config/mid360.yaml`**

常用几项：

| 参数 | 默认 | 说明 |
|---|---|---|
| `point_filter_num` | 3 | 每 3 个点取 1 个，调大更省 CPU |
| `filter_size_surf/map` | 0.5 | 体素降采样尺寸 |
| `extrinsic_est_en` | true | 在线估计 IMU-LiDAR 外参 |
| `blind` | 0.5 | 近处盲区（米） |
| `pcd_save_en` | true | ⚠️ 会把**整段飞行的点云全存内存**，长航时前改成 `false` |

> ⚠️ `time_sync_en` **不是**"雷达↔主机时间同步"开关，它是 FAST_LIO 内部的软件偏移估计，**别乱开**。雷达时钟问题见 [`DATA_FLOW.md` §8 R8](DATA_FLOW.md)。

---

## 5. 胶水脚本 `scouting_bridge` 是干什么的

**为什么需要它**：FAST_LIO 说它的世界系叫 `camera_init`、PX4 说它的本地系叫 `map`、ego-planner 又希望里程计叫 `world` 且坐标系要 ENU。三个组件各说各话，**`scouting_bridge` 就是中间那个翻译**。

包里有 5 个节点（`setup.py` 的 `console_scripts`），参数统一在 `config/real_drone.yaml`：

| 节点 | 一句话作用 | 输入 | 输出 |
|---|---|---|---|
| **`vision_pose_relay`** | 把 SLAM 位姿喂给飞控，让 PX4 知道自己在哪 | `/Odometry` | `/mavros/vision_pose/pose` |
| **`slam_odom_relay`** | 把 SLAM 里程计换个坐标系名给规划器用 | `/Odometry` | `/odom_world` |
| **`tf_broadcaster`** | 发 3 条静态 TF，告诉系统各传感器装在哪 | 参数 | `/tf_static` |
| **`tag_pose_relay`** | 把相机系里的 tag 换算成世界系坐标 | TF + `/Odometry` | `/tag_detection/pose`、`/tag_detection/id` |
| **`tag_mission_controller`** | 比赛跑完后扫描 tag、PID 精准降落 | 上面那些 + MAVROS | `/mavros/setpoint_raw/local` 等 |

几个容易困惑的点：

- **`vision_pose_relay` 只是"抄一遍"**：把 `/Odometry` 的 pose 原样复制，只把 `frame_id` 改成 `camera_init`，**时间戳保持不动**（PX4 靠它做延迟补偿，重新盖时间戳会害了它）。
- **`slam_odom_relay` 只改字符串，不做坐标变换**：`camera_init` → `world`、`body` → `base_link`，数值一个不动。所以"`world` 系"并不是真的另一个坐标系。
- **`tf_broadcaster` 只发静态边，且刻意不发 `odom → base_link`**。原因：一个 frame 只能有一个父节点，多发一条会把 TF 树劈成两棵互不相连的树，`tag_pose_relay` 的查找就会全部失败。**不要"顺手"加回去。**
- **`tag_pose_relay` 不是简单地转发**：它要手工把 `T_world→body`（来自 `/Odometry`）和 `T_body→tag`（TF 链）复合起来，因为 `world` 这个系在 TF 里不存在。多个 tag 同时可见时它发布的是**质心**（自然对准降落板中心）。

更细的每个节点行为见 [`DATA_FLOW.md` §3.3](DATA_FLOW.md)。

---

## 6. 数据流一张图

```
                    ┌─────────────┐
 Livox Mid-360 ────►│ FAST_LIO    │─── /Odometry ──┬─► vision_pose_relay ─► /mavros/vision_pose/pose ─► PX4 EKF2
   /livox/lidar     │ (SLAM)      │                │
   /livox/imu       └─────────────┘                ├─► slam_odom_relay ───► /odom_world ─┐
                                                   │                                     │
                                                   └─ /cloud_registered_body ────┐       │
                                                                                 ▼       ▼
                                                                            ┌──────────────┐
                                                                            │ ego-planner  │
                                                                            └──────┬───────┘
                                                                   planning/bspline│
                                                                                   ▼
                                                                            ┌──────────────┐
                                                                            │ traj_server  │─► /position_cmd
                                                                            └──────────────┘        │
                                                                                                    ▼
 USB 相机 ─► /camera_down/image_raw/compressed ─► apriltag_ros ─TF─┐        ┌──────────────┐
                                                                   ├───────►│  cmd_bridge  │─► /mavros/setpoint_raw/local ─► PX4
                                            tag_pose_relay ◄───────┘        └──────────────┘
                                                   │
                                                   └─► /tag_detection/{pose,id} ─► tag_mission_controller ─► PX4
```

完整的、带频率和消息类型的版本见 [`DATA_FLOW.md` §2/§4](DATA_FLOW.md)，可交互版见 [`UAV_DATAFLOW.html`](UAV_DATAFLOW.html)。

---

## 7. 改完怎么验证

```bash
# ── 雷达 ──
ping 192.168.1.178                      # 网络通不通
ros2 topic hz /livox/lidar              # ~10 Hz
ros2 topic hz /livox/imu

# ── SLAM ──
ros2 topic hz /Odometry                 # ~10 Hz
ros2 topic hz /cloud_registered_body    # ~10 Hz

# ── 相机与 tag ──
ros2 topic hz /camera_down/image_raw/compressed   # ~30 Hz
ros2 topic echo /tag_detection/id       # 对着 tag 应出 ID，拿开应出 -1

# ── TF 树 ──
ros2 run tf2_tools view_frames          # 生成 frames.pdf，检查是不是一棵树
ros2 run tf2_ros tf2_echo body camera_down        # 应出数（验证外参 B）
ros2 run tf2_ros tf2_echo body tag36h11:0         # 应出数（tag 可见时）

# ── 到 PX4 ──
ros2 topic echo /mavros/state --once    # connected: true
ros2 topic hz /mavros/vision_pose/pose  # ~10 Hz

# ── 规划 ──
ros2 topic hz /position_cmd             # ~100 Hz
ros2 topic echo /mission/stage          # 任务阶段

# ── 起飞前必做：规划系和控制系对不对齐 ──
ros2 topic echo /Odometry                   --field pose.pose --once
ros2 topic echo /mavros/local_position/odom --field pose.pose --once
# 各自算 yaw 后相减：|Δψ| < 2° 且 |Δt| < 0.2 m 才算安全
```

---

## 8. 新手最容易踩的 8 个坑

| # | 坑 | 后果 |
|---|---|---|
| 1 | **以为只有一个"外参"** | 动了雷达只改了一处，另一次没改 → tag 位姿或 SLAM 建图歪掉。见 §4.1 |
| 2 | **`tf_broadcaster` 的角度单位是弧度不是度** | `camera_down_pitch: 90` 会让相机坐标系转 90 弧度，tag 完全找不到 |
| 3 | **只改 JSON 里的雷达 IP，没配主机网卡地址** | `ping` 不通，`/livox/lidar` 一条数据没有 |
| 4 | **把 `pixel_format` 改成 `mjpeg2rgb`** | 裸图被 DDS 丢光，相机"有话题没数据" |
| 5 | **改航点时改了 launch 里的 `mission/takeoff_height`** | 那是死参数，不会有任何效果。要改 YAML 的 `defaults` |
| 6 | **只改 `max_vel` 其中一处** | `manager`/`optimization`/`bspline` 三组必须一致 |
| 7 | **"顺手"给 `tf_broadcaster` 加一条 `odom → base_link`** | TF 树被劈成两棵，`tag_pose_relay` 全部查找失败，降落链路失效 |
| 8 | **新增 yaml/launch 后没重新 `colcon build`** | `glob()` 是构建时求值的，新文件不会进 `install/` |

---

## 9. 术语表

| 术语 | 含义 |
|---|---|
| `camera_init` | FAST_LIO 的世界系，**第一次激光扫描时**原点归零，yaw 来自 IMU 初始姿态 |
| `body` | FAST_LIO 的机体（IMU）坐标系 |
| `base_link` | 机体中心坐标系（PX4 习惯的命名） |
| `world` | `slam_odom_relay` 贴的标签，**数值上等于 `camera_init`**，TF 树里不存在 |
| `map` | **MAVROS/PX4 的本地世界系**，EKF 初始化时归零。TF 树里也不存在（MAVROS 的 tf 输出是关的） |
| ENU / NED | 东-北-天 / 北-东-地。**本系统全链路用 ENU**，MAVROS 内部自己转 NED，不要手工转 |
| OFFBOARD | PX4 的模式：位置/姿态由外部（MAVROS）给指令 |
| EKF2 | PX4 的状态估计器。本系统把 SLAM 位姿喂给它做融合 |
| 胶水包 | 指 `scouting_bridge`，负责在 SLAM / PX4 / 规划器之间做话题和坐标系的翻译 |

---

## 10. 进一步阅读

| 想了解 | 看 |
|---|---|
| 完整话题清单、TF 树、任务时序 | [`DATA_FLOW.md`](DATA_FLOW.md) |
| 可交互数据流图（可缩放/搜索/聚焦） | [`UAV_DATAFLOW.html`](UAV_DATAFLOW.html) |
| 已知风险点与验证方法 | [`DATA_FLOW.md` §8](DATA_FLOW.md) |
| 上线前必须确认的外部配置 | [`DATA_FLOW.md` §8 R1](DATA_FLOW.md)（MAVROS 侧） |
| 最大的隐性风险（规划系≠控制系） | [`DATA_FLOW.md` §8.1 R10](DATA_FLOW.md) |
