# Navigation Layer

English | [中文](./README.zh-CN.md)

---

![Cover](./image/cover.gif)


This layer contains Nav2 configuration, the path-tracking controller, obstacle input conversion, safety gates, GPS/replay helpers, and navigation monitoring tools.

## Packages

- `swerve_navigation`: Nav2 launch files, parameters, behavior trees, and helper scripts.
- `nmpc_controller`: Nav2 controller plugin implementing acados-based NMPC for the swerve chassis.
- `pointcloud_to_laserscan`: Converts 3D point clouds into 2D `LaserScan` data for Nav2 costmaps.

## Main Launch Entrypoints

- `swerve_navigation/launch/full_system.launch.py`
- `swerve_navigation/launch/mapping.launch.py`
- `swerve_navigation/launch/vla_replay.launch.py`
- `swerve_navigation/launch/vla_base_path_tracking.launch.py`

The navigation layer consumes maps, localization, and obstacle data, and outputs `/cmd_vel`.

## License

This package is licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.

For details, please refer to the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Acknowledgments

This package is part of the OpenFlex full-body humanoid robot platform ecosystem, developed specifically for research and industrial applications in the humanoid robotics field.

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.
**Chengdu Changshu Robotics Co., Ltd.**

| Contact | Information |
|---------|-------------|
| 📧 Email | openarmrobot@gmail.com |
| 📱 Phone/WeChat | +86-17746530375 |
| 🌐 Website | https://openarmx.com/ |
| 🌐 Docs | http://docs.openarmx.com/ |
| 📍 Address | Tianjin Xiqing District · Daochao Robot Experience Base (City of Tomorrow) · Tianjin Humanoid Robot Center |
| 👤 Contact Person | Mr. Wang |
