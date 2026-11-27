# swerve_navigation

English | [中文](./README-CN.md)

---

`swerve_navigation` is the main OpenFlex entry package for mapping, localization, navigation, and navigation-related utility nodes.

For user-facing mapping and navigation workflows, use this package instead of launching lower-level packages such as `pgo`, `icp_registration`, `pointcloud_to_laserscan`, or `nmpc_controller` directly.

## Package Role

Main directories:

| Directory | Role |
|---|---|
| `launch/` | Mapping, full navigation, and VLA replay launch files |
| `config/` | Nav2, collision monitor, and replay configuration |
| `behavior_trees/` | Nav2 behavior trees |
| `rviz/` | RViz debug layouts |
| `scripts/` | Odometry muxing, point cloud relay, safety gate, VLA helpers, and monitoring tools |

## Main Launch Files

| Launch | Role |
|---|---|
| `launch/mapping.launch.py` | Start the mapping stack with Livox, FAST-LIVO bridge, PGO, chassis control, and RViz |
| `launch/full_system.launch.py` | Start the full navigation stack with localization, Nav2, collision monitor, point cloud projection, and RViz |
| `launch/vla_replay.launch.py` | Start support nodes for VLA policy replay |
| `launch/vla_base_path_tracking.launch.py` | Bridge VLA base path output into Nav2 `FollowPath` |

## Mapping Workflow

The examples below use `your_map` as the map name:

```bash
export MAP_NAME=your_map
```

### 1. Prepare For Mapping

Before mapping, you can optionally verify that the chassis moves correctly:

```bash
ros2 launch swerve_bringup swerve_drive.launch.py
```

In another terminal, start keyboard teleoperation:

```bash
ros2 run swerve_bringup swerve_teleop.py
```

After this check, stop `swerve_drive.launch.py` to avoid starting the chassis control stack twice. For actual mapping, `mapping.launch.py` already starts the chassis control nodes; if keyboard driving is needed, run only `swerve_teleop.py` in another terminal.

Create the 3D map directory:

```bash
mkdir -p "$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME"
```

The directory must exist before calling `/pgo/save_maps`; otherwise the save service returns an error.

### 2. Start Mapping

```bash
ros2 launch swerve_navigation mapping.launch.py
```

Common optional arguments:

| Argument | Default | Meaning |
|---|---|---|
| `use_rviz` | `true` | Start RViz |
| `use_camera` | `false` | Start D435 camera for visual fusion workflows |
| `steering_can_interface` | `can5` | Steering CAN interface |
| `driving_can_interface` | `can4` | Driving CAN interface |
| `mapping_max_wheel_speed` | `2.0` | Wheel speed limit during mapping |
| `mapping_wheel_accel_limit` | `1.2` | Wheel acceleration limit during mapping |

For live mapping visualization, watch `/pgo/global_cloud` and `/fastlio2/body_cloud` in RViz.

### 3. Save The 3D Map

After mapping, call the PGO save service:

```bash
mkdir -p "$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME"

ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME', save_patches: true}"
```

Expected output:

```text
~/MID_360_nav/openflex_maps/3d/your_map/
├── map.pcd
├── poses.txt
├── patches/
└── sc_data/
```

### 4. View The 3D Point Cloud Map

View the saved `map.pcd`:

```bash
ros2 launch pgo view_saved_map.launch.py \
  pcd_path:="$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME/map.pcd"
```

For live map inspection, `mapping.launch.py` already starts RViz. Use `/pgo/global_cloud` or `/fastlio2/body_cloud`.

### 5. Convert 3D Map To 2D Navigation Map

This workflow converts a **3D PCD map to a 2D Nav2 map**, not the other way around.

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

Output:

```text
~/MID_360_nav/openflex_maps/2d/your_map/
├── map.yaml
└── map.pgm
```

### 6. Optional Map Editing

Use the GUI helper to generate and edit the 2D map:

```bash
ros2 run pgo pcd_to_nav2_map_gui
```

## Full Navigation

Start navigation:

```bash
ros2 launch swerve_navigation full_system.launch.py \
  pcd_path:="$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME/map.pcd" \
  map_yaml:="$HOME/MID_360_nav/openflex_maps/2d/$MAP_NAME/map.yaml" \
  initial_pose:="[0.0,0.0,0.0,0.0,0.0,0.0]"
```

Required inputs:

| Argument | Meaning |
|---|---|
| `pcd_path` | 3D PCD map used by ICP localization |
| `map_yaml` | 2D Nav2 occupancy map |

Common optional arguments:

| Argument | Default | Meaning |
|---|---|---|
| `use_rviz` | `true` | Start RViz |
| `initial_pose` | `[0,0,0,0,0,0]` | ICP initial pose `[x,y,z,roll,pitch,yaw]` |
| `nav_controller_max_wheel_speed` | `1.2` | Navigation wheel speed limit |
| `nav_controller_wheel_accel_limit` | `1.0` | Navigation wheel acceleration limit |

After startup, set the initial pose in RViz with `2D Pose Estimate` or provide `initial_pose` in the launch command.

## Main Runtime Topics

| Topic | Producer | Consumer |
|---|---|---|
| `/fastlio2/lio_odom` | FAST-LIVO bridge | `safe_odom_mux`, NMPC |
| `/fastlio2/body_cloud` | FAST-LIVO bridge | ICP, point cloud relay |
| `/odom` | `swerve_drive_controller` | `safe_odom_mux` |
| `/odom_safe` | `safe_odom_mux.py` | Nav2 |
| `/livox/body_cloud_nav2` | `pointcloud_qos_relay.py` | `pointcloud_to_laserscan` |
| `/scan` | `pointcloud_to_laserscan_node` | Nav2 local costmap |
| `/cmd_vel_safe` | `cmd_vel_safety_gate.py` | `swerve_drive_controller` |
| `/initialpose` | RViz or launch initial pose | `icp_registration_node` |

## Utility Scripts

| Script | Role |
|---|---|
| `safe_odom_mux.py` | Switch between LIO odometry and wheel odometry, then publish `/odom_safe` and TF |
| `cmd_vel_safety_gate.py` | Force zero command while localization hold is active |
| `pointcloud_qos_relay.py` | Restamp, filter, and republish body cloud for Nav2 |
| `laser_scan_qos_relay.py` | Relay LaserScan with Nav2-friendly QoS |
| `local_window_marker.py` | Publish a local window marker for RViz |
| `nav_monitor_gui.py` | Monitor navigation command and safety topics |
| `future_odom_pose_tracker.py` | Track VLA future-odom pose targets |
| `vla_path_to_nav2_follow_path.py` | Bridge VLA base path messages into Nav2 `FollowPath` |

## Checks

```bash
ros2 topic list
ros2 topic hz /odom_safe
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo --once /cmd_vel_safe
```

If navigation does not move, check `/cmd_vel`, `/cmd_vel_safe_in`, `/cmd_vel_safe`, localization status, and whether `collision_monitor` is holding the robot.
