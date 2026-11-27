# nmpc_controller

[English](./README.md) | 中文

---

`nmpc_controller` 是 OpenFlex 舵轮底盘的 Nav2 控制器插件，用于路径跟踪和局部避障控制。

该插件实现 `nav2_core::Controller` 接口，内部调用 acados 生成的求解器，输出适合舵轮底盘的机体系速度：

```text
cmd_vel: vx, vy, omega
```

## 包作用

主要文件：

| 文件 | 作用 |
|---|---|
| `src/nmpc_controller.cpp` | Nav2 控制器插件主实现 |
| `src/acados_wrapper.cpp` | acados 生成求解器封装 |
| `generated/` | 舵轮模型对应的 acados C 求解器 |
| `nmpc_controller_plugin.xml` | pluginlib 插件注册 |
| `config/nmpc_params.yaml` | 独立参数参考 |
| `script/generate_solver.py` | 求解器生成脚本 |

插件类名：

```text
nmpc_controller::NmpcController
```

## 正常使用方式

OpenFlex 正常导航时由 `swerve_navigation/config/nav2_params.yaml` 加载本插件：

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nmpc_controller::NmpcController"
```

完整导航入口：

```bash
ros2 launch swerve_navigation full_system.launch.py \
  pcd_path:=/path/to/map.pcd \
  map_yaml:=/path/to/map.yaml
```

## 模型与控制量

控制器使用 6 维状态：

```text
x, y, theta, vx, vy, omega
```

控制量为加速度：

```text
ax, ay, alpha
```

支持能力：

| 能力 | 作用 |
|---|---|
| 舵轮原生 `vx/vy/omega` 跟踪 | 支持横移，不把底盘退化为差速模型 |
| 代价地图 NMPC 惩罚 | 避免预测轨迹穿过高代价区域 |
| 局部 A* 修复 | 近场障碍导致路径不顺时辅助重连 |
| 起步/终点朝向控制 | 改善起步转向和终点姿态收敛 |
| LIO 里程计订阅 | 可利用短期速度反馈 |

## 关键参数

常用参数配置在 Nav2 控制器插件 ID 下，通常为 `FollowPath`：

| 参数组 | 示例 |
|---|---|
| 预测范围 | `horizon_steps`, `horizon_time` |
| 状态权重 | `Q_px`, `Q_py`, `Q_theta`, `Q_vx`, `Q_vy`, `Q_omega` |
| 控制权重 | `R_ax`, `R_ay`, `R_alpha` |
| 终端权重 | `Q_e_px`, `Q_e_py`, `Q_e_theta` |
| 速度限制 | `vx_max`, `vx_min`, `vy_max`, `omega_max` |
| 加速度限制 | `ax_max`, `ay_max`, `alpha_max` |
| 避障行为 | `costmap_weight`, `local_astar_enabled`, `local_astar_cost_threshold` |
| 朝向行为 | `rotate_to_heading_threshold`, `final_rotate_yaw_goal_tolerance` |

参考文件：

```text
src/openflex_chassis/navigation_layer/swerve_navigation/config/nav2_params.yaml
src/openflex_chassis/navigation_layer/nmpc_controller/config/nmpc_params.yaml
```

## 构建说明

本包链接 acados、HPIPM、BLASFEO 和 qpOASES：

```bash
colcon build --packages-select nmpc_controller
```

`CMakeLists.txt` 使用 deb 安装的 `/opt/openflex/acados`。构建本包前需要先安装 `openflex-acados_*.deb`。

## 检查命令

```bash
ros2 pkg plugins --package nmpc_controller
ros2 param get /controller_server FollowPath.horizon_steps
ros2 topic echo /cmd_vel
```

如果 Nav2 controller server 无法加载插件，优先检查 `nmpc_controller_plugin.xml` 是否已安装，以及 acados 相关动态库是否在运行时库路径中。
