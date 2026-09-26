# swerve_navigation

[English](./README.md) | 中文

---

`swerve_navigation` 是 OpenFlex 面向用户的建图、定位、导航和导航辅助工具主入口包。

建图、定位、导航等完整流程建议从本包启动，不建议直接把 `pgo`、`icp_registration`、`pointcloud_to_laserscan`、`nmpc_controller` 等底层包作为主入口。

## 包作用

主要目录：

| 目录 | 作用 |
|---|---|
| `launch/` | 建图、完整导航、VLA 回放相关启动文件 |
| `config/` | Nav2、碰撞监控、回放等配置 |
| `behavior_trees/` | Nav2 行为树 |
| `rviz/` | RViz 调试布局 |
| `scripts/` | 里程计复用、点云转发、安全门、VLA 辅助和监控工具 |

## 主要启动文件

| Launch | 作用 |
|---|---|
| `launch/mapping.launch.py` | 启动 Livox、FAST-LIVO 兼容桥接、PGO、底盘控制和 RViz 建图链路 |
| `launch/full_system.launch.py` | 启动定位、Nav2、碰撞监控、点云投影和 RViz 完整导航链路 |
| `launch/vla_replay.launch.py` | 启动 VLA 策略回放所需辅助节点 |
| `launch/vla_base_path_tracking.launch.py` | 将 VLA 底盘路径输出桥接到 Nav2 `FollowPath` |

## 建图流程

以下示例统一使用地图名 `your_map`：

```bash
export MAP_NAME=your_map
```

### 1. 建图准备

建图前可先单独检查底盘运动是否正常：

```bash
ros2 launch swerve_bringup swerve_drive.launch.py
```

另开终端启动键盘控制：

```bash
ros2 run swerve_bringup swerve_teleop.py
```

检查完成后停止 `swerve_drive.launch.py`，避免和 `mapping.launch.py` 重复启动底盘控制链路。正式建图时，`mapping.launch.py` 已经会启动底盘运控节点；如果需要键盘遥控，只需另开终端运行 `swerve_teleop.py`。

创建 3D 地图目录：

```bash
mkdir -p "$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME"
```

注意：保存地图前目录必须已经存在，否则 `/pgo/save_maps` 会返回失败。

### 2. 启动建图节点

```bash
ros2 launch swerve_navigation mapping.launch.py
```

常用可选参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `use_rviz` | `true` | 是否启动 RViz |
| `use_camera` | `false` | 是否启动 D435 相机，用于视觉融合流程 |
| `steering_can_interface` | `can5` | 转向电机 CAN 接口 |
| `driving_can_interface` | `can4` | 驱动电机 CAN 接口 |
| `mapping_max_wheel_speed` | `2.0` | 建图时轮速上限 |
| `mapping_wheel_accel_limit` | `1.2` | 建图时轮速变化率上限 |

实时建图时，RViz 中主要关注 `/pgo/global_cloud` 和 `/fastlio2/body_cloud`。

### 3. 保存 3D 地图

建图完成后调用 PGO 保存服务：

```bash
mkdir -p "$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME"

ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME', save_patches: true}"
```

成功后会生成：

```text
~/MID_360_nav/openflex_maps/3d/your_map/
├── map.pcd
├── poses.txt
├── patches/
└── sc_data/
```

### 4. 查看 3D 点云地图

查看已保存的 `map.pcd`：

```bash
ros2 launch pgo view_saved_map.launch.py \
  pcd_path:="$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME/map.pcd"
```

如果只看实时建图点云，`mapping.launch.py` 已经会打开 RViz，主要看 `/pgo/global_cloud` 或 `/fastlio2/body_cloud`。

### 5. 3D 地图转 2D 导航地图

这里是 **3D PCD 转 2D Nav2 地图**，不是 2D 转 3D。

```bash
mkdir -p "$HOME/MID_360_nav/openflex_maps/2d/$MAP_NAME"

ros2 run pgo pcd_to_nav2_map \
  --pcd "$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME/map.pcd" \
  --out-dir "$HOME/MID_360_nav/openflex_maps/2d/$MAP_NAME" \
  --poses "$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME/poses.txt" \
  --resolution 0.05 \
  --z-min 0.00 \
  --z-max 2.30 \
  --obstacle-z-min 0.10 \
  --obstacle-z-max 1.40 \
  --inflation-radius 0.10
```

输出：

```text
~/MID_360_nav/openflex_maps/2d/your_map/
├── map.yaml
└── map.pgm
```

### 6. 可选修图

可以使用图形界面生成和修图：

```bash
ros2 run pgo pcd_to_nav2_map_gui
```

## 完整导航

启动导航：

```bash
ros2 launch swerve_navigation full_system.launch.py \
  pcd_path:="$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME/map.pcd" \
  map_yaml:="$HOME/MID_360_nav/openflex_maps/2d/$MAP_NAME/map.yaml" \
  initial_pose:="[0.0,0.0,0.0,0.0,0.0,0.0]"
```

必填输入：

| 参数 | 含义 |
|---|---|
| `pcd_path` | ICP 定位使用的 3D PCD 地图 |
| `map_yaml` | Nav2 使用的 2D 栅格地图 |

常用可选参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `use_rviz` | `true` | 是否启动 RViz |
| `initial_pose` | `[0,0,0,0,0,0]` | ICP 初始位姿 `[x,y,z,roll,pitch,yaw]` |
| `nav_controller_max_wheel_speed` | `1.2` | 导航时轮速上限 |
| `nav_controller_wheel_accel_limit` | `1.0` | 导航时轮速变化率上限 |

启动后可在 RViz 中使用 `2D Pose Estimate` 指定初始位姿，也可以直接通过 `initial_pose` 参数提供初始猜测。

## 主要运行话题

| 话题 | 发布者 | 订阅者 |
|---|---|---|
| `/fastlio2/lio_odom` | FAST-LIVO 兼容桥接 | `safe_odom_mux`、NMPC |
| `/fastlio2/body_cloud` | FAST-LIVO 兼容桥接 | ICP、点云转发节点 |
| `/odom` | `swerve_drive_controller` | `safe_odom_mux` |
| `/odom_safe` | `safe_odom_mux.py` | Nav2 |
| `/livox/body_cloud_nav2` | `pointcloud_qos_relay.py` | `pointcloud_to_laserscan` |
| `/scan` | `pointcloud_to_laserscan_node` | Nav2 局部代价地图 |
| `/cmd_vel_safe` | `cmd_vel_safety_gate.py` | `swerve_drive_controller` |
| `/initialpose` | RViz 或 launch 初始位姿参数 | `icp_registration_node` |

## 辅助脚本

| 脚本 | 作用 |
|---|---|
| `safe_odom_mux.py` | 在 LIO 里程计和轮式里程计之间安全切换，并发布 `/odom_safe` 和 TF |
| `cmd_vel_safety_gate.py` | 定位保持期间强制输出零速度 |
| `pointcloud_qos_relay.py` | 为 Nav2 重打时间戳、过滤并转发车体点云 |
| `laser_scan_qos_relay.py` | 使用适合 Nav2 的 QoS 转发 LaserScan |
| `local_window_marker.py` | 发布 RViz 局部窗口 marker |
| `nav_monitor_gui.py` | 监控导航命令链路和安全话题 |
| `future_odom_pose_tracker.py` | 跟踪 VLA future-odom 位姿目标 |
| `vla_path_to_nav2_follow_path.py` | 将 VLA 底盘路径消息桥接到 Nav2 `FollowPath` |

## 检查命令

```bash
ros2 topic list
ros2 topic hz /odom_safe
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo --once /cmd_vel_safe
```

如果导航不运动，优先检查 `/cmd_vel`、`/cmd_vel_safe_in`、`/cmd_vel_safe`、定位状态，以及 `collision_monitor` 是否正在保持停车。
