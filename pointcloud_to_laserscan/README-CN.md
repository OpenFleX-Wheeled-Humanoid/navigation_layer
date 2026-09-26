# pointcloud_to_laserscan

[English](./README.md) | 中文

---

`pointcloud_to_laserscan` 用于将 3D `sensor_msgs/msg/PointCloud2` 点云转换为 2D `sensor_msgs/msg/LaserScan`，供 Nav2 这类 2D 导航栈使用。

在 OpenFlex 中，本包由 `swerve_navigation` 集成调用，把激光/LIO 输出的车体点云投影成 `/scan`，再交给 Nav2 局部代价地图的障碍物层使用。

## 包作用

主要组件：

| 组件 | 作用 |
|---|---|
| `pointcloud_to_laserscan_node` | PointCloud2 转 LaserScan |
| `laserscan_to_pointcloud_node` | LaserScan 转 PointCloud2 |
| `dummy_pointcloud_publisher` | 发布测试用虚拟点云 |

主要启动文件：

| Launch | 作用 |
|---|---|
| `launch/pointcloud_to_laserscan_launch.py` | 面向 OpenFlex 的 PointCloud2 转 LaserScan 启动文件 |
| `launch/sample_pointcloud_to_laserscan_launch.py` | 上游示例转换启动文件 |
| `launch/sample_laserscan_to_pointcloud_launch.py` | 上游示例反向转换启动文件 |

## OpenFlex 使用方式

正常导航时不建议单独启动本包，推荐使用完整导航入口：

```bash
ros2 launch swerve_navigation full_system.launch.py \
  pcd_path:=/path/to/map.pcd \
  map_yaml:=/path/to/map.yaml
```

`full_system.launch.py` 中会自动启动本节点：

| 话题 | 方向 | 说明 |
|---|---|---|
| `/livox/body_cloud_nav2` | 输入 | 经过 `pointcloud_qos_relay.py` 过滤和重打时间戳的车体点云 |
| `/scan` | 输出 | Nav2 代价地图使用的 2D LaserScan |

## 单独调试

需要单独验证点云投影时可运行：

```bash
ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py
```

该启动文件默认重映射：

```text
cloud_in -> /fastlio2/body_cloud
scan     -> /scan
```

## 关键参数

| 参数 | OpenFlex 常用值 | 含义 |
|---|---:|---|
| `target_frame` | `base_link` 或 `mid360_link` | 投影使用的目标坐标系 |
| `transform_tolerance` | `0.05` | TF 查询容差 |
| `min_height` | `-0.15` | 接收点的最低 Z 高度 |
| `max_height` | `0.40` | 接收点的最高 Z 高度 |
| `angle_min` | `-3.14159` | 扫描起始角 |
| `angle_max` | `3.14159` | 扫描结束角 |
| `angle_increment` | `0.0043` | 角分辨率 |
| `range_min` | `0.45` | 最小距离 |
| `range_max` | `10.0` | 最大距离 |
| `use_inf` | `true` | 无回波射线是否使用 `inf` |

## 检查命令

```bash
ros2 topic hz /scan
ros2 topic echo --once /scan
ros2 topic info /scan
ros2 run tf2_ros tf2_echo base_link livox_frame
```

如果 `/scan` 为空或频率异常，优先检查输入点云是否发布、目标坐标系是否存在，以及高度过滤参数是否把点云全部滤掉。
