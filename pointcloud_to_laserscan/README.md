# pointcloud_to_laserscan

English | [中文](./README-CN.md)

---

`pointcloud_to_laserscan` converts 3D `sensor_msgs/msg/PointCloud2` data into 2D `sensor_msgs/msg/LaserScan` data for 2D navigation stacks such as Nav2.

In OpenFlex, this package is used by `swerve_navigation` to project the LiDAR/LIO body cloud into `/scan`, which is then consumed by the Nav2 local costmap obstacle layer.

## Package Role

Main components:

| Component | Role |
|---|---|
| `pointcloud_to_laserscan_node` | Convert PointCloud2 to LaserScan |
| `laserscan_to_pointcloud_node` | Convert LaserScan back to PointCloud2 |
| `dummy_pointcloud_publisher` | Publish synthetic point clouds for simple tests |

Main launch files:

| Launch | Role |
|---|---|
| `launch/pointcloud_to_laserscan_launch.py` | OpenFlex-oriented PointCloud2 to LaserScan launch |
| `launch/sample_pointcloud_to_laserscan_launch.py` | Upstream-style sample conversion launch |
| `launch/sample_laserscan_to_pointcloud_launch.py` | Upstream-style reverse conversion sample |

## OpenFlex Usage

For normal robot navigation, do not launch this package directly. Use:

```bash
ros2 launch swerve_navigation full_system.launch.py \
  pcd_path:=/path/to/map.pcd \
  map_yaml:=/path/to/map.yaml
```

`full_system.launch.py` starts this node with:

| Topic | Direction | Description |
|---|---|---|
| `/livox/body_cloud_nav2` | input | Filtered and restamped body cloud from `pointcloud_qos_relay.py` |
| `/scan` | output | 2D LaserScan used by Nav2 costmaps |

## Standalone Launch

For isolated debugging:

```bash
ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py
```

The standalone launch remaps:

```text
cloud_in -> /fastlio2/body_cloud
scan     -> /scan
```

## Important Parameters

| Parameter | Typical OpenFlex Value | Meaning |
|---|---:|---|
| `target_frame` | `base_link` or `mid360_link` | Frame used for projection |
| `transform_tolerance` | `0.05` | TF lookup tolerance |
| `min_height` | `-0.15` | Minimum accepted Z height |
| `max_height` | `0.40` | Maximum accepted Z height |
| `angle_min` | `-3.14159` | Scan start angle |
| `angle_max` | `3.14159` | Scan end angle |
| `angle_increment` | `0.0043` | Angular resolution |
| `range_min` | `0.45` | Minimum range |
| `range_max` | `10.0` | Maximum range |
| `use_inf` | `true` | Use `inf` for rays without returns |

## Checks

```bash
ros2 topic hz /scan
ros2 topic echo --once /scan
ros2 topic info /scan
ros2 run tf2_ros tf2_echo base_link livox_frame
```

If `/scan` is empty or stale, check that the input cloud is publishing, the target frame exists in TF, and the height limits are not filtering out all points.
