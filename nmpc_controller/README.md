# nmpc_controller

English | [中文](./README-CN.md)

---

`nmpc_controller` provides the Nav2 controller plugin used by OpenFlex for swerve-drive path tracking.

The plugin implements `nav2_core::Controller`, uses an acados-generated solver, and outputs body-frame velocity commands for a swerve chassis:

```text
cmd_vel: vx, vy, omega
```

## Package Role

Main files:

| File | Role |
|---|---|
| `src/nmpc_controller.cpp` | Nav2 controller plugin implementation |
| `src/acados_wrapper.cpp` | Wrapper around the generated acados solver |
| `generated/` | Generated acados C solver for the swerve model |
| `nmpc_controller_plugin.xml` | Pluginlib registration |
| `config/nmpc_params.yaml` | Standalone parameter reference |
| `script/generate_solver.py` | Solver generation script |

Plugin class:

```text
nmpc_controller::NmpcController
```

## Normal Usage

For normal OpenFlex navigation, this package is loaded through `swerve_navigation/config/nav2_params.yaml`:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nmpc_controller::NmpcController"
```

Start the full navigation stack with:

```bash
ros2 launch swerve_navigation full_system.launch.py \
  pcd_path:=/path/to/map.pcd \
  map_yaml:=/path/to/map.yaml
```

## Model and Control

The controller uses a 6-state model:

```text
x, y, theta, vx, vy, omega
```

and acceleration controls:

```text
ax, ay, alpha
```

It supports:

| Feature | Purpose |
|---|---|
| Swerve-native `vx/vy/omega` tracking | Allows lateral motion instead of forcing differential-drive behavior |
| Costmap-aware NMPC cost | Penalizes predicted motion through high-cost cells |
| Local A* repair | Helps reconnect around near-field obstacles |
| Rotate-to-heading logic | Improves start and final heading behavior |
| LIO odometry subscription | Uses short-term velocity feedback when available |

## Important Parameters

Common parameters are configured under the Nav2 controller plugin ID, usually `FollowPath`:

| Parameter Group | Examples |
|---|---|
| Horizon | `horizon_steps`, `horizon_time` |
| State weights | `Q_px`, `Q_py`, `Q_theta`, `Q_vx`, `Q_vy`, `Q_omega` |
| Control weights | `R_ax`, `R_ay`, `R_alpha` |
| Terminal weights | `Q_e_px`, `Q_e_py`, `Q_e_theta` |
| Velocity limits | `vx_max`, `vx_min`, `vy_max`, `omega_max` |
| Acceleration limits | `ax_max`, `ay_max`, `alpha_max` |
| Obstacle behavior | `costmap_weight`, `local_astar_enabled`, `local_astar_cost_threshold` |
| Heading behavior | `rotate_to_heading_threshold`, `final_rotate_yaw_goal_tolerance` |

See:

```text
src/openflex_chassis/navigation_layer/swerve_navigation/config/nav2_params.yaml
src/openflex_chassis/navigation_layer/nmpc_controller/config/nmpc_params.yaml
```

## Build Notes

The build links against acados, HPIPM, BLASFEO, and qpOASES:

```bash
colcon build --packages-select nmpc_controller
```

`CMakeLists.txt` uses the deb-installed acados at `/opt/openflex/acados`. Install `openflex-acados_*.deb` before building this package.

## Checks

```bash
ros2 pkg plugins --package nmpc_controller
ros2 param get /controller_server FollowPath.horizon_steps
ros2 topic echo /cmd_vel
```

If the controller server fails to load the plugin, check that `nmpc_controller_plugin.xml` is installed and that the acados libraries are visible in the runtime library path.
