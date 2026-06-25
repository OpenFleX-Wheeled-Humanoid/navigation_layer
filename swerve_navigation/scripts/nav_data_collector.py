#!/usr/bin/env python3
"""
Nav Data Collector — 后台数据采集 + 自动分析 + 调参建议
用法: python3 nav_data_collector.py
      Ctrl+C 停止采集并生成分析报告

采集数据:
  - /odom_safe: 机器人位姿与轮式里程计速度
  - /fastlio2/lio_odom: LIO 里程计（延迟检测）
  - /cmd_vel: Nav2 velocity_smoother 输出
  - /cmd_vel_safe_in: collision_monitor 输出
  - /cmd_vel_safe: safety_gate 最终输出
  - /plan: 全局路径
  - /local_costmap/costmap: 代价地图
  - /joint_states: 转向角
  - /predicted_path (controller_server): NMPC 预测轨迹
"""
import math
import os
import signal
import sys
import time
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState

LOG_BASE = os.path.expanduser("~/openflex_all/openflex_ws/log")

STEER_JOINTS = ["fl_steering_joint", "fr_steering_joint",
                "bl_steering_joint", "br_steering_joint"]
STEER_LABELS = ["FL", "FR", "BL", "BR"]


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class NavDataCollector(Node):
    def __init__(self):
        super().__init__("nav_data_collector")
        self.get_logger().info("Nav Data Collector 启动，Ctrl+C 停止并生成报告")

        # --- 时间基准 ---
        self.start_time = time.time()

        # --- 状态 ---
        self.robot_x = self.robot_y = self.robot_yaw = 0.0
        self.odom_vx = self.odom_vy = self.odom_wz = 0.0
        self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0
        self.safe_in_vx = self.safe_in_vy = self.safe_in_wz = 0.0
        self.safe_vx = self.safe_vy = self.safe_wz = 0.0
        self.steer_angles = [0.0] * 4
        self.path_points = []
        self.has_odom = False
        self.has_cmd = False
        self.navigating = False

        # --- LIO 延迟监测 ---
        self.last_lio_stamp = None
        self.lio_delays = []

        # --- 统计缓冲 ---
        self.cte_list = []
        self.heading_err_list = []
        self.speed_list = []
        self.cmd_vx_list = []
        self.cmd_vy_list = []
        self.cmd_wz_list = []
        self.odom_vx_list = []
        self.odom_vy_list = []
        self.vel_track_err_list = []     # |cmd - odom| 速度跟踪误差
        self.collision_slow_count = 0    # collision_monitor 减速次数
        self.collision_stop_count = 0    # collision_monitor 停车次数
        self.safety_gate_zero_count = 0  # safety_gate 发零值次数
        self.replan_count = 0
        self.steer_large_angle_count = 0  # 大角度转向次数
        self.vel_clamp_count = 0         # NMPC 速度 clamp 次数
        self.vel_sudden_change_count = 0 # 速度突变次数
        self.prev_cmd_vx = 0.0
        self.prev_cmd_vy = 0.0
        self.prev_safe_vx = 0.0

        # 滑动窗口用于检测
        self.cmd_window = deque(maxlen=20)  # 最近 20 帧 cmd_vel
        self.safe_window = deque(maxlen=20)
        self.frame_count = 0

        # --- 录制文件 ---
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.rec_dir = os.path.join(LOG_BASE, f"monitor_{ts}")
        os.makedirs(self.rec_dir, exist_ok=True)
        csv_path = os.path.join(self.rec_dir, "tracking_data.csv")
        self.csv = open(csv_path, "w")
        self.csv.write(
            "time,robot_x,robot_y,robot_yaw,"
            "nearest_path_x,nearest_path_y,cte,heading_err,"
            "cmd_vx,cmd_vy,cmd_wz,"
            "safe_in_vx,safe_in_vy,safe_in_wz,"
            "safe_vx,safe_vy,safe_wz,"
            "odom_vx,odom_vy,odom_wz,"
            "speed,replan_count,"
            "steer_fl,steer_fr,steer_bl,steer_br,"
            "lio_delay\n")
        self.get_logger().info(f"数据录制到: {self.rec_dir}")

        # --- 订阅 ---
        self.create_subscription(Odometry, "/odom_safe", self._odom_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_safe_in", self._safe_in_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_safe", self._safe_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        # LIO odom (SensorDataQoS)
        self.create_subscription(
            Odometry, "/fastlio2/lio_odom", self._lio_odom_cb,
            qos_profile_sensor_data)

        # Plan (transient local)
        plan_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Path, "/plan", self._plan_cb, plan_qos)

        # 定时采样 (10Hz)
        self.create_timer(0.1, self._sample_tick)

        # 实时打印 (2Hz)
        self.create_timer(0.5, self._print_tick)

    # ==================== callbacks ====================
    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_vy = msg.twist.twist.linear.y
        self.odom_wz = msg.twist.twist.angular.z
        self.has_odom = True

    def _cmd_cb(self, msg):
        prev_vx = self.cmd_vx
        prev_vy = self.cmd_vy
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z
        self.has_cmd = True

        # 检测速度突变 (> 0.15 m/s 变化)
        dvx = abs(self.cmd_vx - prev_vx)
        dvy = abs(self.cmd_vy - prev_vy)
        if dvx > 0.15 or dvy > 0.15:
            self.vel_sudden_change_count += 1

        self.cmd_window.append((self.cmd_vx, self.cmd_vy, self.cmd_wz))

    def _safe_in_cb(self, msg):
        self.safe_in_vx = msg.linear.x
        self.safe_in_vy = msg.linear.y
        self.safe_in_wz = msg.angular.z

        # 检测 collision_monitor 介入
        cmd_speed = math.hypot(self.cmd_vx, self.cmd_vy)
        safe_in_speed = math.hypot(msg.linear.x, msg.linear.y)
        if cmd_speed > 0.02:
            if safe_in_speed < 0.01:
                self.collision_stop_count += 1
            elif safe_in_speed < cmd_speed * 0.5:
                self.collision_slow_count += 1

    def _safe_cb(self, msg):
        self.safe_vx = msg.linear.x
        self.safe_vy = msg.linear.y
        self.safe_wz = msg.angular.z

        # 检测 safety_gate 发零值
        safe_in_speed = math.hypot(self.safe_in_vx, self.safe_in_vy)
        safe_speed = math.hypot(msg.linear.x, msg.linear.y)
        if safe_in_speed > 0.02 and safe_speed < 0.01:
            self.safety_gate_zero_count += 1

        self.safe_window.append((msg.linear.x, msg.linear.y, msg.angular.z))

    def _js_cb(self, msg):
        for i, name in enumerate(STEER_JOINTS):
            if name in msg.name:
                idx = msg.name.index(name)
                angle = msg.position[idx]
                self.steer_angles[i] = angle
                if abs(angle) > 1.0:  # > 57 deg
                    self.steer_large_angle_count += 1

    def _lio_odom_cb(self, msg):
        now = self.get_clock().now()
        msg_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        delay_ms = (now.nanoseconds - msg_stamp.nanoseconds) / 1e6
        self.last_lio_stamp = delay_ms
        self.lio_delays.append(delay_ms)

    def _plan_cb(self, msg):
        self.replan_count += 1
        self.path_points = []
        for p in msg.poses:
            self.path_points.append((
                p.pose.position.x, p.pose.position.y,
                yaw_from_quat(p.pose.orientation)))
        if self.path_points:
            self.navigating = True

    # ==================== analysis ====================
    def cross_track_error(self):
        if not self.path_points or not self.has_odom:
            return 0.0, 0.0, 0.0, 0.0
        min_dist = float("inf")
        best_idx = 0
        for i, (px, py, _) in enumerate(self.path_points):
            d = math.hypot(self.robot_x - px, self.robot_y - py)
            if d < min_dist:
                min_dist = d
                best_idx = i
        _, _, path_yaw = self.path_points[best_idx]
        heading_err = normalize_angle(self.robot_yaw - path_yaw)
        nx, ny = self.path_points[best_idx][0], self.path_points[best_idx][1]
        return min_dist, heading_err, nx, ny

    # ==================== periodic ====================
    def _sample_tick(self):
        if not self.has_odom:
            return

        self.frame_count += 1
        t = time.time() - self.start_time
        cte, he, nx, ny = self.cross_track_error()
        speed = math.hypot(self.cmd_vx, self.cmd_vy)
        lio_delay = self.last_lio_stamp if self.last_lio_stamp is not None else -1.0

        # 累积统计（仅导航中）
        if self.navigating and speed > 0.01:
            self.cte_list.append(cte)
            self.heading_err_list.append(abs(he))
            self.speed_list.append(speed)
            self.cmd_vx_list.append(self.cmd_vx)
            self.cmd_vy_list.append(self.cmd_vy)
            self.cmd_wz_list.append(abs(self.cmd_wz))
            self.odom_vx_list.append(self.odom_vx)
            self.odom_vy_list.append(self.odom_vy)

            # 速度跟踪误差
            vel_err = math.hypot(self.cmd_vx - self.odom_vx,
                                  self.cmd_vy - self.odom_vy)
            self.vel_track_err_list.append(vel_err)

        # 写 CSV
        sa = self.steer_angles
        self.csv.write(
            f"{t:.2f},{self.robot_x:.4f},{self.robot_y:.4f},{self.robot_yaw:.4f},"
            f"{nx:.4f},{ny:.4f},{cte:.4f},{he:.4f},"
            f"{self.cmd_vx:.4f},{self.cmd_vy:.4f},{self.cmd_wz:.4f},"
            f"{self.safe_in_vx:.4f},{self.safe_in_vy:.4f},{self.safe_in_wz:.4f},"
            f"{self.safe_vx:.4f},{self.safe_vy:.4f},{self.safe_wz:.4f},"
            f"{self.odom_vx:.4f},{self.odom_vy:.4f},{self.odom_wz:.4f},"
            f"{speed:.4f},{self.replan_count},"
            f"{sa[0]:.4f},{sa[1]:.4f},{sa[2]:.4f},{sa[3]:.4f},"
            f"{lio_delay:.1f}\n")
        self.csv.flush()

    def _print_tick(self):
        if not self.has_odom:
            print("\r等待 /odom_safe ...", end="", flush=True)
            return

        cte, he, _, _ = self.cross_track_error()
        cte_cm = cte * 100
        he_deg = math.degrees(he)
        speed = math.hypot(self.cmd_vx, self.cmd_vy)
        safe_speed = math.hypot(self.safe_vx, self.safe_vy)
        lio = f"{self.last_lio_stamp:.0f}ms" if self.last_lio_stamp else "N/A"

        # 检测 collision_monitor 当前状态
        cm_state = "OK"
        if speed > 0.02 and safe_speed < 0.01:
            cm_state = "STOP"
        elif speed > 0.02 and safe_speed < speed * 0.5:
            cm_state = "SLOW"

        status = (
            f"\r[{self.frame_count:5d}] "
            f"CTE:{cte_cm:5.1f}cm  "
            f"Head:{he_deg:+5.1f}°  "
            f"Spd:{speed:.2f}/{safe_speed:.2f}m/s  "
            f"LIO:{lio}  "
            f"CM:{cm_state:4s}  "
            f"Replan:{self.replan_count}  "
            f"VelJump:{self.vel_sudden_change_count}"
        )
        print(status, end="", flush=True)

    # ==================== 报告生成 ====================
    def generate_report(self):
        self.csv.close()

        report_path = os.path.join(self.rec_dir, "analysis_report.txt")
        n = len(self.cte_list)
        elapsed = time.time() - self.start_time

        with open(report_path, "w") as f:
            f.write("=" * 70 + "\n")
            f.write("  Nav Data Collector — 自动分析报告\n")
            f.write(f"  时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"  采集时长: {elapsed:.1f}s  |  有效帧数: {n}\n")
            f.write("=" * 70 + "\n\n")

            if n < 10:
                f.write("数据不足，无法生成有效分析。\n")
                print(f"\n\n报告已保存到: {report_path}")
                return report_path

            # --- 路径跟踪 ---
            import statistics
            cte_mean = statistics.mean(self.cte_list) * 100  # cm
            cte_max = max(self.cte_list) * 100
            cte_p95 = sorted(self.cte_list)[int(n * 0.95)] * 100
            cte_median = statistics.median(self.cte_list) * 100
            he_mean = math.degrees(statistics.mean(self.heading_err_list))
            he_max = math.degrees(max(self.heading_err_list))

            f.write("1. 路径跟踪精度\n")
            f.write("-" * 40 + "\n")
            f.write(f"  CTE 均值:    {cte_mean:.2f} cm\n")
            f.write(f"  CTE 中位数:  {cte_median:.2f} cm\n")
            f.write(f"  CTE P95:     {cte_p95:.2f} cm\n")
            f.write(f"  CTE 最大:    {cte_max:.2f} cm\n")
            f.write(f"  航向误差均值: {he_mean:.2f}°\n")
            f.write(f"  航向误差最大: {he_max:.2f}°\n\n")

            # --- 速度统计 ---
            spd_mean = statistics.mean(self.speed_list)
            spd_max = max(self.speed_list)
            vx_mean = statistics.mean(self.cmd_vx_list)
            vy_usage = sum(1 for v in self.cmd_vy_list if abs(v) > 0.02) / n * 100
            wz_mean = statistics.mean(self.cmd_wz_list)

            f.write("2. 速度分析\n")
            f.write("-" * 40 + "\n")
            f.write(f"  平均速度:     {spd_mean:.3f} m/s\n")
            f.write(f"  最大速度:     {spd_max:.3f} m/s\n")
            f.write(f"  平均 vx:      {vx_mean:.3f} m/s (期望 {0.30:.2f})\n")
            f.write(f"  vy 使用率:    {vy_usage:.1f}%\n")
            f.write(f"  平均 |wz|:    {wz_mean:.3f} rad/s\n")
            f.write(f"  速度突变次数: {self.vel_sudden_change_count}\n\n")

            # --- 速度跟踪 ---
            vel_err_mean = statistics.mean(self.vel_track_err_list) * 100  # cm/s
            vel_err_max = max(self.vel_track_err_list) * 100

            f.write("3. 速度跟踪 (cmd vs odom)\n")
            f.write("-" * 40 + "\n")
            f.write(f"  跟踪误差均值: {vel_err_mean:.1f} cm/s\n")
            f.write(f"  跟踪误差最大: {vel_err_max:.1f} cm/s\n\n")

            # --- 安全层 ---
            f.write("4. 安全层统计\n")
            f.write("-" * 40 + "\n")
            f.write(f"  collision_monitor 减速次数: {self.collision_slow_count}\n")
            f.write(f"  collision_monitor 停车次数: {self.collision_stop_count}\n")
            f.write(f"  safety_gate 零值输出次数:  {self.safety_gate_zero_count}\n")
            f.write(f"  重规划次数:                {self.replan_count}\n\n")

            # --- LIO 延迟 ---
            if self.lio_delays:
                lio_mean = statistics.mean(self.lio_delays)
                lio_max = max(self.lio_delays)
                lio_p95 = sorted(self.lio_delays)[int(len(self.lio_delays) * 0.95)]
                f.write("5. LIO 里程计延迟\n")
                f.write("-" * 40 + "\n")
                f.write(f"  延迟均值: {lio_mean:.1f} ms\n")
                f.write(f"  延迟 P95: {lio_p95:.1f} ms\n")
                f.write(f"  延迟最大: {lio_max:.1f} ms\n\n")
            else:
                f.write("5. LIO 里程计延迟\n")
                f.write("-" * 40 + "\n")
                f.write("  未收到 /fastlio2/lio_odom 数据\n\n")

            # --- 转向 ---
            f.write("6. 转向统计\n")
            f.write("-" * 40 + "\n")
            f.write(f"  大角度转向次数 (>57°): {self.steer_large_angle_count}\n\n")

            # ==================== 调参建议 ====================
            f.write("=" * 70 + "\n")
            f.write("  调参建议\n")
            f.write("=" * 70 + "\n\n")

            suggestions = []

            # CTE 分析
            if cte_mean > 8.0:
                suggestions.append(
                    f"[CTE偏大] CTE均值={cte_mean:.1f}cm，建议:\n"
                    f"  - 提高 Q_px/Q_py (当前 75→90+)\n"
                    f"  - 提高 Q_e_px/Q_e_py (当前 95→110+)\n"
                    f"  - 降低 R_ax/R_ay (当前 0.08/0.06→0.05/0.04)\n")
            elif cte_mean > 5.0:
                suggestions.append(
                    f"[CTE可优化] CTE均值={cte_mean:.1f}cm，建议:\n"
                    f"  - 适度提高 Q_px/Q_py (当前 75→85)\n"
                    f"  - 降低 R_ax (当前 0.08→0.06)\n")
            elif cte_mean < 3.0:
                suggestions.append(
                    f"[CTE良好] CTE均值={cte_mean:.1f}cm，路径跟踪精度优秀\n")

            # 速度突变
            if self.vel_sudden_change_count > 50:
                suggestions.append(
                    f"[速度突变频繁] {self.vel_sudden_change_count}次，建议:\n"
                    f"  - 提高 R_ax/R_ay 控制代价 (当前 0.08/0.06→0.12/0.10)\n"
                    f"  - 降低 velocity_smoother max_accel\n"
                    f"  - 或增大 curve_speed_reduction_gain\n")

            # vy 使用率
            if vy_usage < 5:
                suggestions.append(
                    f"[vy低利用] vy使用率仅{vy_usage:.0f}%，横向修正能力未充分利用\n"
                    f"  - 降低 Q_vy (当前 3.0→2.0) 放松横向惩罚\n")
            elif vy_usage > 40:
                suggestions.append(
                    f"[vy过度使用] vy使用率{vy_usage:.0f}%，可能导致侧滑\n"
                    f"  - 提高 Q_vy (当前 3.0→5.0)\n")

            # 速度利用
            if spd_mean < 0.15:
                suggestions.append(
                    f"[速度偏低] 平均速度仅{spd_mean:.2f}m/s (期望0.30)\n"
                    f"  - 检查 collision_monitor 是否频繁触发\n"
                    f"  - 检查转向阈值是否过小\n"
                    f"  - 提高 desired_linear_vel\n")

            # collision_monitor
            if self.collision_stop_count > 10:
                suggestions.append(
                    f"[频繁急停] collision_monitor停车{self.collision_stop_count}次\n"
                    f"  - 检查 StopZone 大小是否合理\n"
                    f"  - 提高 max_points 门槛\n"
                    f"  - 检查点云过滤 (地面/自身点是否清理)\n")
            if self.collision_slow_count > 30:
                suggestions.append(
                    f"[频繁减速] collision_monitor减速{self.collision_slow_count}次\n"
                    f"  - 缩小 SlowZone 前缘距离\n"
                    f"  - 提高 SlowZone max_points\n")

            # safety_gate
            if self.safety_gate_zero_count > 5:
                suggestions.append(
                    f"[safety_gate异常] 零值输出{self.safety_gate_zero_count}次\n"
                    f"  - 检查 cmd_vel 链路是否正确串联\n"
                    f"  - collision_monitor → /cmd_vel_safe_in → safety_gate → /cmd_vel_safe\n")

            # LIO 延迟
            if self.lio_delays:
                lio_p95_val = sorted(self.lio_delays)[int(len(self.lio_delays) * 0.95)]
                if lio_p95_val > 200:
                    suggestions.append(
                        f"[LIO延迟大] P95={lio_p95_val:.0f}ms\n"
                        f"  - 检查 FAST-LIVO2 计算负载\n"
                        f"  - 考虑增大 transform_tolerance\n")

            # 航向误差
            if he_mean > 8.0:
                suggestions.append(
                    f"[航向偏差大] 均值={he_mean:.1f}°\n"
                    f"  - 提高 Q_theta (当前 15→20)\n"
                    f"  - 提高 Q_e_theta (当前 22→28)\n"
                    f"  - 增大 reference_heading_smoothing_gain\n")

            # 速度跟踪
            if vel_err_mean > 8.0:
                suggestions.append(
                    f"[速度跟踪差] cmd-odom差{vel_err_mean:.1f}cm/s\n"
                    f"  - 检查底盘响应延迟\n"
                    f"  - 考虑 velocity_smoother 用 CLOSED_LOOP\n")

            if not suggestions:
                suggestions.append("当前参数表现良好，暂无需调整。\n")

            for i, s in enumerate(suggestions):
                f.write(f"{i+1}. {s}\n")

            f.write("\n" + "=" * 70 + "\n")
            f.write(f"数据文件: {self.rec_dir}/tracking_data.csv\n")
            f.write("=" * 70 + "\n")

        print(f"\n\n报告已保存到: {report_path}")
        # 同时打印到终端
        with open(report_path) as f:
            print(f.read())

        return report_path


def main():
    rclpy.init()
    node = NavDataCollector()

    def shutdown_handler(sig, frame):
        print("\n\n停止采集，生成报告...")
        node.generate_report()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
