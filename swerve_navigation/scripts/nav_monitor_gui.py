#!/usr/bin/env python3
"""
Nav Monitor GUI — 舵轮导航实时监控与数据录制
用法: python3 nav_monitor_gui.py
依赖: rclpy, tkinter (Python 自带)
"""
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

import tkinter as tk
from tkinter import ttk, messagebox

LOG_BASE = os.path.expanduser("~/openflex_all/openflex_ws/log")

STEER_JOINTS = ["fl_steering_joint", "fr_steering_joint",
                "bl_steering_joint", "br_steering_joint"]
STEER_LABELS = ["FL", "FR", "BL", "BR"]


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavMonitorNode(Node):
    def __init__(self):
        super().__init__("nav_monitor_gui")

        # --- state ---
        self.robot_x = self.robot_y = self.robot_yaw = 0.0
        self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0
        self.safe_vx = self.safe_vy = self.safe_wz = 0.0
        self.steer_angles = [0.0] * 4          # rad
        self.path_points = []                   # [(x,y,yaw), ...]
        self.replan_count = 0
        self.robot_trail = []                   # [(x,y), ...]
        self.has_odom = False
        self.start_time = time.time()

        # recording
        self.recording = False
        self.rec_file = None
        self.rec_count = 0
        self.rec_dir = None

        # --- subscriptions ---
        self.create_subscription(Odometry, "/odom_safe", self._odom_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_safe", self._safe_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        plan_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Path, "/plan", self._plan_cb, plan_qos)

    # ---------- callbacks ----------
    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.has_odom = True
        # trail (keep last 2000 points)
        self.robot_trail.append((self.robot_x, self.robot_y))
        if len(self.robot_trail) > 2000:
            self.robot_trail = self.robot_trail[-2000:]

    def _cmd_cb(self, msg):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z

    def _safe_cb(self, msg):
        self.safe_vx = msg.linear.x
        self.safe_vy = msg.linear.y
        self.safe_wz = msg.angular.z

    def _js_cb(self, msg):
        for i, name in enumerate(STEER_JOINTS):
            if name in msg.name:
                idx = msg.name.index(name)
                self.steer_angles[i] = msg.position[idx]

    def _plan_cb(self, msg):
        self.replan_count += 1
        self.path_points = []
        for p in msg.poses:
            self.path_points.append((
                p.pose.position.x, p.pose.position.y,
                yaw_from_quat(p.pose.orientation)))

    # ---------- analysis ----------
    def cross_track_error(self):
        if not self.path_points:
            return 0.0, 0.0, 0.0, 0.0
        min_dist = float("inf")
        best_idx = 0
        for i, (px, py, _) in enumerate(self.path_points):
            d = math.hypot(self.robot_x - px, self.robot_y - py)
            if d < min_dist:
                min_dist = d
                best_idx = i
        _, _, path_yaw = self.path_points[best_idx]
        heading_err = self.robot_yaw - path_yaw
        while heading_err > math.pi:
            heading_err -= 2 * math.pi
        while heading_err < -math.pi:
            heading_err += 2 * math.pi
        nx, ny = self.path_points[best_idx][0], self.path_points[best_idx][1]
        return min_dist, heading_err, nx, ny

    # ---------- recording ----------
    def start_recording(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.rec_dir = os.path.join(LOG_BASE, f"monitor_{ts}")
        os.makedirs(self.rec_dir, exist_ok=True)
        path = os.path.join(self.rec_dir, "tracking_analysis.csv")
        self.rec_file = open(path, "w")
        self.rec_file.write(
            "time,robot_x,robot_y,robot_yaw,"
            "nearest_path_x,nearest_path_y,cross_track_err,heading_err,"
            "cmd_vx,cmd_vy,cmd_wz,safe_vx,safe_vy,safe_wz,speed,"
            "replan_count,steer_fl,steer_fr,steer_bl,steer_br\n")
        self.rec_count = 0
        self.recording = True
        self.start_time = time.time()

    def stop_recording(self):
        self.recording = False
        if self.rec_file:
            self.rec_file.close()
            self.rec_file = None

    def record_frame(self):
        if not self.recording or not self.rec_file:
            return
        t = time.time() - self.start_time
        cte, he, nx, ny = self.cross_track_error()
        speed = math.hypot(self.cmd_vx, self.cmd_vy)
        sa = self.steer_angles
        self.rec_file.write(
            f"{t:.2f},{self.robot_x:.4f},{self.robot_y:.4f},{self.robot_yaw:.4f},"
            f"{nx:.4f},{ny:.4f},{cte:.4f},{he:.4f},"
            f"{self.cmd_vx:.4f},{self.cmd_vy:.4f},{self.cmd_wz:.4f},"
            f"{self.safe_vx:.4f},{self.safe_vy:.4f},{self.safe_wz:.4f},"
            f"{speed:.4f},{self.replan_count},"
            f"{sa[0]:.4f},{sa[1]:.4f},{sa[2]:.4f},{sa[3]:.4f}\n")
        self.rec_file.flush()
        self.rec_count += 1


class NavMonitorGUI:
    BG = "#1e1e2e"
    FG = "#cdd6f4"
    ACCENT = "#89b4fa"
    RED = "#f38ba8"
    GREEN = "#a6e3a1"
    YELLOW = "#f9e2af"
    SURFACE = "#313244"
    CANVAS_BG = "#181825"

    def __init__(self, node: NavMonitorNode):
        self.node = node

        self.root = tk.Tk()
        self.root.title("Nav Monitor")
        self.root.geometry("860x620")
        self.root.configure(bg=self.BG)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_ui()
        self._tick()
        self.root.mainloop()

    # ==================== UI ====================
    def _build_ui(self):
        # --- top bar ---
        top = tk.Frame(self.root, bg=self.SURFACE, height=40)
        top.pack(fill=tk.X, padx=4, pady=(4, 2))
        top.pack_propagate(False)

        tk.Label(top, text="Nav Monitor", font=("Mono", 14, "bold"),
                 bg=self.SURFACE, fg=self.ACCENT).pack(side=tk.LEFT, padx=8)

        self.rec_btn = tk.Button(top, text="● REC", font=("Mono", 10, "bold"),
                                 bg=self.SURFACE, fg=self.RED, relief=tk.FLAT,
                                 activebackground=self.BG, command=self._toggle_rec)
        self.rec_btn.pack(side=tk.LEFT, padx=6)

        tk.Button(top, text="Reset Trail", font=("Mono", 9),
                  bg=self.SURFACE, fg=self.FG, relief=tk.FLAT,
                  activebackground=self.BG,
                  command=self._reset_trail).pack(side=tk.LEFT, padx=6)

        self.status_label = tk.Label(top, text="等待连接...", font=("Mono", 9),
                                     bg=self.SURFACE, fg=self.YELLOW)
        self.status_label.pack(side=tk.RIGHT, padx=8)

        # --- main area ---
        main = tk.Frame(self.root, bg=self.BG)
        main.pack(fill=tk.BOTH, expand=True, padx=4, pady=2)

        # left panel
        left = tk.Frame(main, bg=self.BG, width=220)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 4))
        left.pack_propagate(False)

        self._build_speed_panel(left)
        self._build_tracking_panel(left)
        self._build_wheel_panel(left)

        # right canvas
        canvas_frame = tk.Frame(main, bg=self.SURFACE)
        canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(canvas_frame, bg=self.CANVAS_BG,
                                highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        # --- bottom bar ---
        bot = tk.Frame(self.root, bg=self.SURFACE, height=28)
        bot.pack(fill=tk.X, padx=4, pady=(2, 4))
        bot.pack_propagate(False)

        self.bot_label = tk.Label(bot, text="", font=("Mono", 9),
                                  bg=self.SURFACE, fg=self.FG)
        self.bot_label.pack(side=tk.LEFT, padx=8)

    def _make_section(self, parent, title):
        frame = tk.LabelFrame(parent, text=title, font=("Mono", 10, "bold"),
                              bg=self.BG, fg=self.ACCENT,
                              labelanchor="nw", padx=6, pady=4)
        frame.pack(fill=tk.X, pady=(0, 4))
        return frame

    def _label_row(self, parent, label_text):
        row = tk.Frame(parent, bg=self.BG)
        row.pack(fill=tk.X, pady=1)
        tk.Label(row, text=label_text, font=("Mono", 10),
                 bg=self.BG, fg=self.FG, width=10, anchor="w").pack(side=tk.LEFT)
        val = tk.Label(row, text="--", font=("Mono", 11, "bold"),
                       bg=self.BG, fg=self.GREEN, anchor="e")
        val.pack(side=tk.RIGHT)
        return val

    def _build_speed_panel(self, parent):
        f = self._make_section(parent, "速度")
        self.lbl_vx = self._label_row(f, "vx")
        self.lbl_vy = self._label_row(f, "vy")
        self.lbl_wz = self._label_row(f, "wz")
        self.lbl_speed = self._label_row(f, "speed")
        self.lbl_safe_diff = self._label_row(f, "safe差异")

    def _build_tracking_panel(self, parent):
        f = self._make_section(parent, "路径跟踪")
        self.lbl_cte = self._label_row(f, "CTE")
        self.lbl_heading = self._label_row(f, "航向误差")
        self.lbl_replan = self._label_row(f, "重规划")

    def _build_wheel_panel(self, parent):
        f = self._make_section(parent, "转向角度")
        self.lbl_wheels = []
        for name in STEER_LABELS:
            self.lbl_wheels.append(self._label_row(f, name))

    # ==================== actions ====================
    def _toggle_rec(self):
        if self.node.recording:
            self.node.stop_recording()
            self.rec_btn.config(text="● REC", fg=self.RED)
            if self.node.rec_dir:
                messagebox.showinfo("保存完成",
                                    f"数据已保存到:\n{self.node.rec_dir}\n"
                                    f"共 {self.node.rec_count} 帧")
        else:
            self.node.start_recording()
            self.rec_btn.config(text="■ STOP", fg=self.GREEN)

    def _reset_trail(self):
        self.node.robot_trail.clear()

    def _on_close(self):
        if self.node.recording:
            self.node.stop_recording()
        self.root.destroy()
        rclpy.shutdown()

    # ==================== tick loop ====================
    def _tick(self):
        try:
            rclpy.spin_once(self.node, timeout_sec=0)
        except Exception:
            pass

        self._update_labels()
        self._update_canvas()
        self.node.record_frame()
        self.root.after(100, self._tick)  # 10 Hz

    def _update_labels(self):
        n = self.node

        # speed
        self.lbl_vx.config(text=f"{n.cmd_vx:+.3f} m/s")
        self.lbl_vy.config(text=f"{n.cmd_vy:+.3f} m/s")
        self.lbl_wz.config(text=f"{n.cmd_wz:+.3f} r/s")
        speed = math.hypot(n.cmd_vx, n.cmd_vy)
        self.lbl_speed.config(text=f"{speed:.3f} m/s")

        # safe diff
        dvx = abs(n.cmd_vx - n.safe_vx)
        dvy = abs(n.cmd_vy - n.safe_vy)
        dwz = abs(n.cmd_wz - n.safe_wz)
        total_diff = dvx + dvy + dwz
        if total_diff < 0.01:
            self.lbl_safe_diff.config(text="OK", fg=self.GREEN)
        else:
            self.lbl_safe_diff.config(
                text=f"{total_diff:.3f}", fg=self.YELLOW)

        # tracking
        cte, he, _, _ = n.cross_track_error()
        cte_cm = cte * 100
        he_deg = math.degrees(he)
        self.lbl_cte.config(text=f"{cte_cm:.1f} cm")
        if cte_cm > 10:
            self.lbl_cte.config(fg=self.RED)
        elif cte_cm > 5:
            self.lbl_cte.config(fg=self.YELLOW)
        else:
            self.lbl_cte.config(fg=self.GREEN)

        self.lbl_heading.config(text=f"{he_deg:+.1f}°")
        self.lbl_replan.config(text=str(n.replan_count))

        # wheels
        for i in range(4):
            deg = math.degrees(n.steer_angles[i])
            self.lbl_wheels[i].config(text=f"{deg:+.1f}°")

        # status
        if n.has_odom:
            self.status_label.config(text="已连接", fg=self.GREEN)
        else:
            self.status_label.config(text="等待 odom...", fg=self.YELLOW)

        # bottom bar
        t = time.time() - n.start_time
        rec_text = f"录制: {n.rec_count} 帧" if n.recording else "未录制"
        self.bot_label.config(
            text=f"位置: ({n.robot_x:.2f}, {n.robot_y:.2f}) | "
                 f"航向: {math.degrees(n.robot_yaw):.1f}° | "
                 f"{rec_text} | 运行: {t:.0f}s")

    def _update_canvas(self):
        c = self.canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10:
            return

        # gather all points for auto-scale
        all_pts = list(self.node.robot_trail)
        if self.node.path_points:
            all_pts += [(x, y) for x, y, _ in self.node.path_points]
        if self.node.has_odom:
            all_pts.append((self.node.robot_x, self.node.robot_y))

        if len(all_pts) < 2:
            c.create_text(w // 2, h // 2, text="等待数据...",
                          fill=self.FG, font=("Mono", 12))
            return

        xs = [p[0] for p in all_pts]
        ys = [p[1] for p in all_pts]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)

        # add margin
        dx = max(xmax - xmin, 0.5)
        dy = max(ymax - ymin, 0.5)
        margin = 0.15
        xmin -= dx * margin
        xmax += dx * margin
        ymin -= dy * margin
        ymax += dy * margin
        dx = xmax - xmin
        dy = ymax - ymin

        # keep aspect ratio
        scale = min((w - 20) / dx, (h - 20) / dy)
        ox = (w - dx * scale) / 2
        oy = (h - dy * scale) / 2

        def to_px(rx, ry):
            px = ox + (rx - xmin) * scale
            py = oy + (ymax - ry) * scale  # flip y
            return px, py

        # grid
        grid_step = 1.0  # 1m
        gx = math.floor(xmin)
        while gx <= math.ceil(xmax):
            px, _ = to_px(gx, 0)
            c.create_line(px, 0, px, h, fill="#303040", dash=(2, 4))
            c.create_text(px + 2, h - 4, text=f"{gx:.0f}m",
                          fill="#505060", font=("Mono", 7), anchor="sw")
            gx += grid_step
        gy = math.floor(ymin)
        while gy <= math.ceil(ymax):
            _, py = to_px(0, gy)
            c.create_line(0, py, w, py, fill="#303040", dash=(2, 4))
            c.create_text(4, py - 2, text=f"{gy:.0f}m",
                          fill="#505060", font=("Mono", 7), anchor="sw")
            gy += grid_step

        # global path (red)
        if self.node.path_points:
            coords = []
            for px, py, _ in self.node.path_points:
                coords.extend(to_px(px, py))
            if len(coords) >= 4:
                c.create_line(*coords, fill=self.RED, width=2, smooth=True)

        # robot trail (blue)
        if len(self.node.robot_trail) >= 2:
            coords = []
            # subsample for performance
            trail = self.node.robot_trail
            step = max(1, len(trail) // 500)
            for i in range(0, len(trail), step):
                coords.extend(to_px(trail[i][0], trail[i][1]))
            if len(coords) >= 4:
                c.create_line(*coords, fill=self.ACCENT, width=2, smooth=True)

        # robot position
        if self.node.has_odom:
            rx, ry = to_px(self.node.robot_x, self.node.robot_y)
            r = 8
            c.create_oval(rx - r, ry - r, rx + r, ry + r,
                          fill=self.GREEN, outline="")
            # heading arrow
            yaw = self.node.robot_yaw
            ax = rx + 18 * math.cos(-yaw + math.pi / 2)
            ay = ry + 18 * math.sin(-yaw + math.pi / 2)
            # flip for canvas y
            ax = rx + 18 * math.cos(yaw)
            ay = ry - 18 * math.sin(yaw)
            c.create_line(rx, ry, ax, ay, fill=self.GREEN, width=2,
                          arrow=tk.LAST)

        # legend
        c.create_text(8, 8, text="— 全局路径", fill=self.RED,
                      font=("Mono", 9), anchor="nw")
        c.create_text(8, 22, text="— 机器人轨迹", fill=self.ACCENT,
                      font=("Mono", 9), anchor="nw")
        c.create_text(8, 36, text="● 当前位置", fill=self.GREEN,
                      font=("Mono", 9), anchor="nw")


def main():
    rclpy.init()
    node = NavMonitorNode()
    NavMonitorGUI(node)


if __name__ == "__main__":
    main()
