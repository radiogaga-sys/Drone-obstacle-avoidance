# UAV 自主避障飞行系统

> 基于 **Livox Mid-360 激光雷达 + FAST-LIO 定位 + EGO-Planner 避障 + PX4/MAVROS 控制** 的真机自主飞行系统，支持竞赛航点自动执行与 **AprilTag 精准降落**。

**ROS 2 Humble** · **PX4 / MAVROS** · **Livox Mid-360** · **MIT**

---

## 目录

- [这能做什么](#这能做什么)
- [系统组成](#系统组成)
- [环境要求](#环境要求)
- [快速开始](#快速开始)
- [台架验证（不装桨）](#台架验证不装桨)
- [真机首飞前检查](#真机首飞前检查)
- [故障排查](#故障排查)
- [项目结构](#项目结构)
- [配置速查](#配置速查)
- [已知限制](#已知限制)
- [文档](#文档)
- [上游项目与致谢](#上游项目与致谢)

---

## 这能做什么

- 🛰️ **无 GPS 室内定位**：LiDAR-惯性里程计（FAST-LIO）输出位姿，经外部视觉接口喂入 PX4 EKF2
- 🚁 **自主避障飞行**：EGO-Planner 局部重规划 + 占据栅格地图，输出 100 Hz 位置指令
- 🎯 **竞赛航点自动执行**：A/B/C/D 四段航点，起飞→触发→依次飞完→自动交接
- 🛬 **AprilTag 精准降落**：下视相机识别标签，PID 引导对准后 `AUTO.LAND`
- 🔌 **完整话题/坐标系桥接**：`scouting_bridge` 胶水包负责 SLAM ⇄ PX4 ⇄ 规划器 之间的翻译

### 两条闭环（理解整个系统的关键）

```
A. 定位闭环   FAST_LIO /Odometry ─► vision_pose_relay ─► PX4 EKF2 ─► /mavros/local_position/odom
B. 控制闭环   /Odometry ─► slam_odom_relay ─► /odom_world ─► ego-planner ─► traj_server
                        ─► /position_cmd ─► cmd_bridge ─► PX4
```

---

## 系统组成

| 包 | 角色 | 来源 |
|---|---|---|
| **`scouting_bridge`** | **胶水包**：话题/坐标系桥接、TF 维护、AprilTag 降落任务 | 本仓库自研 |
| `ego_planner` | 规划器 FSM + 占据栅格 + B 样条轨迹 + `traj_server` + `cmd_bridge` | 上游 EGO-Planner-Swarm + 本地改动 |
| `fast_lio` | LiDAR-惯性里程计与建图 | 上游 FAST-LIO2 |
| `livox_ros_driver2` | Mid-360 雷达驱动 | 上游 Livox |
| `apriltag_ros` | AprilTag 检测 + PnP 位姿（输出 TF） | 上游 |
| `usb_cam` | 下视相机驱动（含 MJPEG 直通补丁） | 上游 + 补丁 |

---

## 环境要求

### 硬件

| 设备 | 说明 |
|---|---|
| Livox Mid-360 | 雷达，通过网线接机载计算机 |
| 机载计算机 | 本仓库在 NUC 上验证；需有独立有线网口给雷达 |
| PX4 飞控 | 通过串口/USB 连接，运行 MAVROS |
| 下视 USB 相机 | 用于 AprilTag 降落 |

### 软件

| 依赖 | 用途 |
|---|---|
| ROS 2 Humble | 全部 |
| `mavros` / `mavros_msgs` | 飞控通信 |
| PCL / Eigen | FAST-LIO、EGO-Planner |
| [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)（`/usr/local/include/livox_lidar_*.h`） | 雷达驱动 |
| [apriltag](https://github.com/AprilRobotics/apriltag) | 标签检测 |
| `python3-scipy` | `tag_pose_relay` 的旋转复合 |

> 各包的确切依赖见各自的 `package.xml`。

---

## 快速开始

> ⚠️ **本仓库不包含** MAVROS 的启动文件，也**不含仿真环境**（仿真需要额外的雷达模拟器提供 `/mid360`）。下面是真机路径。

### 步骤 1 · 编译

```bash
cd <工作空间根目录>            # 即含 src/ 的那一层
colcon build --symlink-install
source install/setup.bash
```

| 选项 | 作用 |
|---|---|
| `--symlink-install` | 改 `.py` / `.yaml` / `.launch.py` 后**不用重编**，重启 launch 即可 |
| 新增文件（多加一个 yaml/launch） | ⚠️ 仍然要重新 `colcon build`，因为 `setup.py` 的 `glob()` 是构建时求值的 |

### 步骤 2 · 配通雷达网络（最容易卡住的一步）

先改 **`src/livox_ros_driver2/config/MID360s_config.json`** 里的两个 IP：

```jsonc
"host_net_info": [{ "host_ip": "192.168.1.5", ... }]   // ← 机载计算机网口的地址
"lidar_configs": [{ "ip": "192.168.1.178", ... }]       // ← 雷达的地址（看设备标签）
```

再**真的把网口配上这个地址**（很多人只改了 JSON 忘了这步）：

```bash
ip a                                        # 找到接雷达的那个网口名
sudo ip addr add 192.168.1.5/24 dev <网口名>
ping 192.168.1.178                          # 必须通
colcon build --packages-select livox_ros_driver2   # JSON 是构建时拷贝的，要重编
```

### 步骤 3 · 改最少必要的参数

先用默认值跑一遍，能起来了再按实际硬件微调：

| 改什么 | 文件 | 默认值 |
|---|---|---|
| 雷达 / 主机 IP | `src/livox_ros_driver2/config/MID360s_config.json` | `192.168.1.178` / `192.168.1.5` |
| 相机设备号 | `src/scouting_bridge/config/usb_cam_down.yaml` | `/dev/video0` |
| **传感器外参**（LiDAR/相机装在哪） | `src/scouting_bridge/config/real_drone.yaml` → `tf_broadcaster.*` | 见 [配置速查](#配置速查) |
| **AprilTag 标签边长** | `src/scouting_bridge/config/apriltag_config.yaml` → `size` | `0.165`（米，**必须量你实际打印的标签**） |
| 航点 / 起飞高度 | `src/ego-planner-swarm/config/competition_waypoints.yaml` | — |

### 步骤 4 · 启动

```bash
# 终端 1：先起 MAVROS（本仓库不提供，用你们飞控侧既有的方式）
#         确认 fcu_url 指向真机链路，不能用 SITL 回环地址

# 终端 2：一键启动
ros2 launch scouting_bridge real_drone.launch.py
```

常用启动参数：

```bash
ros2 launch scouting_bridge real_drone.launch.py rviz:=true              # 台架调试开 RViz
ros2 launch scouting_bridge real_drone.launch.py enable_mission:=false   # 只跑 SLAM+规划，不跑比赛任务
ros2 launch scouting_bridge real_drone.launch.py enable_apriltag:=false  # 关掉相机链路
```

另外两个启动文件：

| 文件 | 用途 |
|---|---|
| `real_drone_no_tag.launch.py` | 去掉 AprilTag 链路的精简版 |
| `apriltag_pipeline.launch.py` | **只起相机链路**，单独调试检测器 |

### 步骤 5 · 逐项验证

**按顺序敲，每一步都要有输出才继续下一步：**

| # | 命令 | 期望 |
|---|---|---|
| 1 | `ros2 topic hz /livox/lidar` | ~10 Hz |
| 2 | `ros2 topic hz /Odometry` | ~10 Hz |
| 3 | `ros2 topic hz /cloud_registered_body` | ~10 Hz |
| 4 | `ros2 topic echo /mavros/state --once` | `connected: true` |
| 5 | `ros2 topic hz /mavros/vision_pose/pose` | ~10 Hz |
| 6 | `ros2 topic hz /position_cmd` | ~100 Hz |
| 7 | `ros2 run tf2_ros tf2_echo body camera_down` | 能出数（不是 `two or more unconnected trees`） |

同时另开一个终端看 PX4 是否真的收到了外部视觉：

```bash
ros2 topic echo /mavros/local_position/odom --once      # 位置应随飞机移动而变化
```

---

## 台架验证（不装桨）

**首次跑通请先做这一节，别直接飞。**

```bash
ros2 launch scouting_bridge real_drone.launch.py rviz:=true enable_mission:=false
```

在 RViz 里加这些显示项：

| 显示项 | Topic | 应看到 |
|---|---|---|
| PointCloud2 | `/cloud_registered` | 累积地图 |
| PointCloud2 | `/grid_map/occupancy_inflate` | 障碍占据栅格（含膨胀） |
| Odometry | `/odom_world` | 规划器用的里程计 |
| Path | `/path` | SLAM 轨迹 |

**单独验证相机与标签**（可以只起这一条链路）：

```bash
ros2 launch scouting_bridge apriltag_pipeline.launch.py
ros2 topic hz  /camera_down/image_raw/compressed     # ~30 Hz
ros2 topic echo /tag_detection/id                    # 对着标签应出 ID，移开应出 -1
```

---

## 真机首飞前检查

> 这四项都是**本系统特有的、容易忘的**检查，缺一项都可能出问题。

- [ ] **`fcu_url` 指向真机链路**（不是 SITL 回环），`/mavros/state` 的 `connected: true`
- [ ] **MAVROS 的 `tf.send` 为 `false`** —— 否则它会和 SLAM 抢 `base_link` 的父节点，把 TF 树劈成两棵，**AprilTag 降落会整条失效**
      ```bash
      ros2 topic info /tf --verbose        # publisher 应只有 laser_mapping / tf_broadcaster / apriltag_detector
      ```
- [ ] **规划系与控制系对齐** —— 两路位姿在飞机静止时应几乎重合
      ```bash
      ros2 topic echo /Odometry                   --field pose.pose --once
      ros2 topic echo /mavros/local_position/odom --field pose.pose --once
      # 各自算 yaw 后相减：|Δψ| < 2° 且 |Δt| < 0.2 m 才安全
      ```
- [ ] **场地里没有强磁干扰**，且 PX4 的 `EKF2_EV_CTRL` 已开启视觉的位置与 yaw 融合

---

## 故障排查

| 现象 | 最可能的原因 | 怎么查 / 怎么修 |
|---|---|---|
| `/livox/lidar` 没数据 | 雷达 IP 或**主机网卡地址**没配 | `ping 192.168.1.178`；`ip a` 看接雷达的网口 |
| 改了雷达 IP 不生效 | 没重新编译驱动包 | `colcon build --packages-select livox_ros_driver2` |
| 相机"有话题没数据" | `pixel_format` 被改成了 `mjpeg2rgb` | 改回 `raw_mjpeg`（裸图会被 DDS 丢光） |
| `/tag_detection/id` 一直 `-1` | ①标签 `family`/`size` 不对 ②TF 链断了 ③里程计被判定为陈旧 | 依次查 `apriltag_config.yaml`、`tf2_echo body camera_down`、`ros2 topic delay /Odometry` |
| `tf2_echo` 报 `two or more unconnected trees` | MAVROS 的 tf 在发 `map→base_link` | 在 MAVROS 配置里设 `tf.send: false` |
| 规划器刷 `no odom.` | `/odom_world` 没起来 | `ros2 topic hz /Odometry` → 查 SLAM |
| 飞机不飞 / 停在原地 | 没收到 `/traj_start_trigger`，或 `cmd_bridge` 没起 | `ros2 topic echo /mission/stage`；`ros2 node list \| grep cmd_bridge` |
| 改了起飞高度没效果 | 改的是 launch 里的 `mission/takeoff_height`——**那是死参数** | 改 `competition_waypoints.yaml` 的 `defaults` |
| 速度改了行为不对 | `manager` / `optimization` / `bspline` 三组 `max_vel` 没同步 | 三处必须一起改 |
| RViz 里看不到轨迹/路径 | 那些 marker 的 frame 是 `world`/`map`，TF 树里没有 | 用固定帧 `camera_init` 看点云；轨迹显示问题见[已知限制](#已知限制) |

---

## 项目结构

```
├── src/
│   ├── scouting_bridge/          # 胶水包（自研，最常改这里）
│   │   ├── config/               #   ← 所有可调参数
│   │   ├── launch/               #   ← 启动文件
│   │   └── scouting_bridge/      #   ← 5 个节点
│   ├── ego-planner-swarm/        # 规划器
│   │   └── config/competition_waypoints.yaml
│   ├── FAST_LIO_ROS2/            # LiDAR-惯性里程计
│   │   └── config/mid360.yaml
│   ├── livox_ros_driver2/        # 雷达驱动
│   │   └── config/MID360s_config.json
│   ├── apriltag_ros/             # 标签检测
│   └── usb_cam/                  # 相机驱动
└── docs/
    ├── GETTING_STARTED.md        # 上手指南（"我要改 X 去哪改"）
    ├── DATA_FLOW.md              # 数据流详解 + 风险清单
    └── UAV_DATAFLOW.html         # 可交互数据流图
```

### 胶水包里的 5 个节点

| 节点 | 作用 | 输入 → 输出 |
|---|---|---|
| `vision_pose_relay` | 把 SLAM 位姿喂给飞控 | `/Odometry` → `/mavros/vision_pose/pose` |
| `slam_odom_relay` | 换成规划器要的坐标系名 | `/Odometry` → `/odom_world` |
| `tf_broadcaster` | 发 3 条静态 TF（传感器安装位置） | 参数 → `/tf_static` |
| `tag_pose_relay` | tag 从相机系换算到世界系 | TF + `/Odometry` → `/tag_detection/{pose,id}` |
| `tag_mission_controller` | 扫描 tag + PID 精准降落 | 上列 + MAVROS |

---

## 配置速查

| 我要改… | 改这个文件 |
|---|---|
| **传感器外参** | ⚠️ **两处**：`FAST_LIO_ROS2/config/mid360.yaml` 的 `extrinsic_T/R`（IMU↔LiDAR）+ `scouting_bridge/config/real_drone.yaml` 的 `tf_broadcaster.*`（各传感器↔机体中心，**角度单位是弧度**） |
| 雷达 IP / 网络 | `livox_ros_driver2/config/MID360s_config.json` |
| 相机设备 / 分辨率 | `scouting_bridge/config/usb_cam_down.yaml` |
| 相机内参 | `scouting_bridge/config/camera_down.yaml` |
| AprilTag 家族 / 边长 | `scouting_bridge/config/apriltag_config.yaml` |
| 航点 / 起飞高度 | `ego-planner-swarm/config/competition_waypoints.yaml` |
| 规划器（地图/速度/分辨率） | `scouting_bridge/launch/real_drone.launch.py` 的 `param_overrides` |
| SLAM 参数 | `FAST_LIO_ROS2/config/mid360.yaml` |
| 降落 PID / 超时 / 重试 | `scouting_bridge/launch/real_drone.launch.py` 的 `tag_mission_node` |
| 飞控连接（`fcu_url`） | **不在本仓库**，在飞控侧 MAVROS 启动配置 |

---

## 已知限制

- **仿真链路不在本仓库内**。`mid360_sim.yaml` 期望 `/mid360` 话题，但本仓库没有任何节点发布它；需要额外的雷达模拟器。
- **规划系与控制系不是同一个坐标系**。规划器在 `camera_init` 里算，PX4 在自己的本地系里执行，两者靠"上电同时归零 + EKF2 融合视觉"隐式对齐，**代码里没有任何校验**。这是本系统最大的隐性风险，详见 [`docs/DATA_FLOW.md` §8.1](docs/DATA_FLOW.md)。
- **SLAM 崩溃后不会自动重启**（刻意配置）。无人机会原地悬停等人工接手，不会自动恢复。
- RViz 里 ego-planner 发布的轨迹/路径 marker 用的坐标系（`world` / `map`）不在 TF 树中，**默认显示不出来**。点云和栅格地图不受影响。

---

## 文档

| 文档 | 内容 |
|---|---|
| [`docs/GETTING_STARTED.md`](docs/GETTING_STARTED.md) | 上手指南：环境、启动、"我要改 X 去哪改"、常见坑 |
| [`docs/DATA_FLOW.md`](docs/DATA_FLOW.md) | 数据流详解：全量话题清单、TF 树、任务时序、参数生效链路、**风险点与验证方法** |
| [`docs/UAV_DATAFLOW.html`](docs/UAV_DATAFLOW.html) | 可交互数据流图（浏览器打开，支持缩放/搜索/视图聚焦） |

> **上线前请务必读一遍** [`docs/DATA_FLOW.md` §8 风险点](docs/DATA_FLOW.md)——里面的每一条都给了具体的验证命令。

---

## 上游项目与致谢

本仓库建立在这些优秀开源项目之上：

| 项目 | 用途 |
|---|---|
| [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) | LiDAR-惯性里程计 |
| [EGO-Planner-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) | 避障规划器 |
| [Livox-SDK2 / livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | Mid-360 驱动 |
| [apriltag_ros](https://github.com/christianrauch/apriltag_ros) | 标签检测 |
| [usb_cam](https://github.com/ros-drivers/usb_cam) | 相机驱动 |
| [MAVROS](https://github.com/mavlink/mavros) | PX4 通信 |

---

## License

MIT（各上游子目录遵循其原始许可）
