# 导航层

[English](./README.md) | 中文

---

![封面](./image/cover.gif)


本层负责 Nav2 配置、路径跟踪控制器、障碍物输入转换、安全门、GPS/重放辅助和导航监控工具。

## 包清单

- `swerve_navigation`: Nav2 启动、参数、行为树和导航辅助脚本。
- `nmpc_controller`: Nav2 controller 插件，基于 acados 的舵轮 NMPC 路径跟踪控制器。
- `pointcloud_to_laserscan`: 将 3D 点云转换为 2D LaserScan，供 Nav2 costmap 使用。

## 主要入口

- `swerve_navigation/launch/full_system.launch.py`
- `swerve_navigation/launch/mapping.launch.py`
- `swerve_navigation/launch/vla_replay.launch.py`
- `swerve_navigation/launch/vla_base_path_tracking.launch.py`

## 职责边界

导航层消费地图、定位和传感器障碍物输入，输出 `/cmd_vel`。它不直接驱动电机，也不维护底层 CAN 协议。

## 许可证

本包通过 知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 致谢

本包是 OpenFlex 全身人形机器人平台生态系统的一部分，专为人形机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | https://openarmx.com/ |
| 🌐 文档 | http://docs.openarmx.com/ |
| 📍 地址 | 天津市西青区・稻潮机器人体验基地（明日之城）・天津市人形机器人中心 |
| 👤 联系人 | 王先生 |
