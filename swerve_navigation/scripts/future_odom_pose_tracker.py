#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class FutureOdomPoseTracker(Node):
    """Track VLA future-odom pose targets with closed-loop cmd_vel output."""

    def __init__(self) -> None:
        super().__init__("future_odom_pose_tracker")

        self.declare_parameter("input_path_topic", "/vla/base_path")
        self.declare_parameter("odom_topic", "/fastlio2/lio_odom")
        self.declare_parameter("output_cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("target_timeout", 2.0)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("strict_frame_check", True)

        self.declare_parameter("kx", 0.7)
        self.declare_parameter("ky", 0.7)
        self.declare_parameter("k_yaw", 0.9)
        self.declare_parameter("max_vx", 0.22)
        self.declare_parameter("max_vy", 0.10)
        self.declare_parameter("max_wz", 0.25)
        self.declare_parameter("max_ax", 0.45)
        self.declare_parameter("max_ay", 0.35)
        self.declare_parameter("max_awz", 0.5)

        self.declare_parameter("xy_deadband", 0.008)
        self.declare_parameter("yaw_deadband", 0.015)
        self.declare_parameter("target_filter_alpha", 0.6)
        self.declare_parameter("target_jump_limit_xy", 0.15)
        self.declare_parameter("target_jump_limit_yaw", 0.35)
        self.declare_parameter("diag_log_period", 1.0)

        self.input_path_topic = str(self.get_parameter("input_path_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.output_cmd_topic = str(self.get_parameter("output_cmd_topic").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.target_timeout = float(self.get_parameter("target_timeout").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.strict_frame_check = bool(self.get_parameter("strict_frame_check").value)

        self.kx = float(self.get_parameter("kx").value)
        self.ky = float(self.get_parameter("ky").value)
        self.k_yaw = float(self.get_parameter("k_yaw").value)
        self.max_vx = abs(float(self.get_parameter("max_vx").value))
        self.max_vy = abs(float(self.get_parameter("max_vy").value))
        self.max_wz = abs(float(self.get_parameter("max_wz").value))
        self.max_ax = abs(float(self.get_parameter("max_ax").value))
        self.max_ay = abs(float(self.get_parameter("max_ay").value))
        self.max_awz = abs(float(self.get_parameter("max_awz").value))

        self.xy_deadband = abs(float(self.get_parameter("xy_deadband").value))
        self.yaw_deadband = abs(float(self.get_parameter("yaw_deadband").value))
        self.target_filter_alpha = self._clamp(
            float(self.get_parameter("target_filter_alpha").value), 0.0, 1.0
        )
        self.target_jump_limit_xy = max(0.0, float(self.get_parameter("target_jump_limit_xy").value))
        self.target_jump_limit_yaw = max(0.0, float(self.get_parameter("target_jump_limit_yaw").value))
        self.diag_log_period = max(0.0, float(self.get_parameter("diag_log_period").value))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, qos)
        self.create_subscription(Path, self.input_path_topic, self._on_path, qos)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, qos)

        self.target: Pose2D | None = None
        self.target_frame = ""
        self.target_rx_time = None
        self.latest_odom: Odometry | None = None
        self.odom_rx_time = None
        self.last_cmd = Twist()
        self.last_control_time = self.get_clock().now()
        self.last_diag_time = 0.0
        self.warned_frame_mismatch = False

        period = 1.0 / max(self.control_rate, 1.0)
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            "future odom pose tracker started: "
            f"path={self.input_path_topic} odom={self.odom_topic} "
            f"cmd={self.output_cmd_topic} rate={self.control_rate:.1f}Hz"
        )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.odom_rx_time = self.get_clock().now()

    def _on_path(self, msg: Path) -> None:
        if not msg.poses:
            self.target = None
            self.target_rx_time = None
            self._publish_zero()
            return

        pose_stamped = msg.poses[-1]
        frame_id = msg.header.frame_id or pose_stamped.header.frame_id
        if not frame_id:
            self.get_logger().warning("Dropping future odom target without frame_id")
            return

        odom_frame = self.latest_odom.header.frame_id if self.latest_odom is not None else ""
        if odom_frame and frame_id != odom_frame:
            message = f"Dropping future odom target with frame {frame_id}; odom frame is {odom_frame}"
            if self.strict_frame_check:
                if not self.warned_frame_mismatch:
                    self.get_logger().error(message)
                    self.warned_frame_mismatch = True
                return
            if not self.warned_frame_mismatch:
                self.get_logger().warning(message)
                self.warned_frame_mismatch = True

        raw = Pose2D(
            float(pose_stamped.pose.position.x),
            float(pose_stamped.pose.position.y),
            self._yaw_from_quaternion(pose_stamped.pose.orientation),
        )
        self.target = self._condition_target(raw)
        self.target_frame = frame_id
        self.target_rx_time = self.get_clock().now()

    def _condition_target(self, raw: Pose2D) -> Pose2D:
        if self.target is None:
            return raw

        prev = self.target
        dx = raw.x - prev.x
        dy = raw.y - prev.y
        dist = math.hypot(dx, dy)
        if self.target_jump_limit_xy > 0.0 and dist > self.target_jump_limit_xy:
            scale = self.target_jump_limit_xy / max(dist, 1e-9)
            raw = Pose2D(
                prev.x + dx * scale,
                prev.y + dy * scale,
                raw.yaw,
            )

        dyaw = self._normalize_angle(raw.yaw - prev.yaw)
        if self.target_jump_limit_yaw > 0.0 and abs(dyaw) > self.target_jump_limit_yaw:
            raw = Pose2D(
                raw.x,
                raw.y,
                self._normalize_angle(prev.yaw + math.copysign(self.target_jump_limit_yaw, dyaw)),
            )

        alpha = self.target_filter_alpha
        if alpha >= 1.0:
            return raw
        if alpha <= 0.0:
            return prev
        return Pose2D(
            prev.x + alpha * (raw.x - prev.x),
            prev.y + alpha * (raw.y - prev.y),
            self._normalize_angle(prev.yaw + alpha * self._normalize_angle(raw.yaw - prev.yaw)),
        )

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        dt = max((now - self.last_control_time).nanoseconds * 1e-9, 1e-3)
        self.last_control_time = now

        if self.target is None or self.target_rx_time is None:
            self._publish_zero()
            return
        if self.latest_odom is None or self.odom_rx_time is None:
            self._publish_zero()
            return

        target_age = (now - self.target_rx_time).nanoseconds * 1e-9
        odom_age = (now - self.odom_rx_time).nanoseconds * 1e-9
        if target_age > self.target_timeout or odom_age > self.odom_timeout:
            self._publish_zero()
            return

        current = self._odom_pose(self.latest_odom)
        cmd, diag = self._compute_cmd(current, self.target, dt)
        self.cmd_pub.publish(cmd)
        self.last_cmd = cmd
        self._log_diag(current, self.target, cmd, diag)

    def _odom_pose(self, msg: Odometry) -> Pose2D:
        pose = msg.pose.pose
        return Pose2D(
            float(pose.position.x),
            float(pose.position.y),
            self._yaw_from_quaternion(pose.orientation),
        )

    def _compute_cmd(self, current: Pose2D, target: Pose2D, dt: float) -> tuple[Twist, tuple[float, float, float, float]]:
        dx_odom = target.x - current.x
        dy_odom = target.y - current.y
        cos_yaw = math.cos(current.yaw)
        sin_yaw = math.sin(current.yaw)
        err_x = cos_yaw * dx_odom + sin_yaw * dy_odom
        err_y = -sin_yaw * dx_odom + cos_yaw * dy_odom
        err_yaw = self._normalize_angle(target.yaw - current.yaw)

        raw_xy_error = math.hypot(err_x, err_y)
        raw_yaw_error = err_yaw

        if raw_xy_error < self.xy_deadband:
            err_x = 0.0
            err_y = 0.0
        if abs(raw_yaw_error) < self.yaw_deadband:
            err_yaw = 0.0

        cmd = Twist()
        cmd.linear.x = self._clamp(self.kx * err_x, -self.max_vx, self.max_vx)
        cmd.linear.y = self._clamp(self.ky * err_y, -self.max_vy, self.max_vy)
        cmd.angular.z = self._clamp(self.k_yaw * err_yaw, -self.max_wz, self.max_wz)
        self._limit_accel(cmd, dt)
        return cmd, (raw_xy_error, raw_yaw_error, math.hypot(err_x, err_y), err_yaw)

    def _limit_accel(self, cmd: Twist, dt: float) -> None:
        cmd.linear.x = self._limit_rate(cmd.linear.x, self.last_cmd.linear.x, self.max_ax, dt)
        cmd.linear.y = self._limit_rate(cmd.linear.y, self.last_cmd.linear.y, self.max_ay, dt)
        cmd.angular.z = self._limit_rate(cmd.angular.z, self.last_cmd.angular.z, self.max_awz, dt)

    def _limit_rate(self, value: float, previous: float, limit: float, dt: float) -> float:
        if limit <= 0.0:
            return value
        delta = self._clamp(value - previous, -limit * dt, limit * dt)
        return previous + delta

    def _publish_zero(self) -> None:
        zero = Twist()
        self.cmd_pub.publish(zero)
        self.last_cmd = zero

    def _log_diag(self, current: Pose2D, target: Pose2D, cmd: Twist, diag: tuple[float, float, float, float]) -> None:
        if self.diag_log_period <= 0.0:
            return
        now_s = self._now_s()
        if now_s - self.last_diag_time < self.diag_log_period:
            return
        self.last_diag_time = now_s
        raw_xy_error, raw_yaw_error, active_xy_error, active_yaw_error = diag
        self.get_logger().info(
            "future odom tracker diag: "
            f"pos=({current.x:.2f},{current.y:.2f},{math.degrees(current.yaw):.1f}deg) "
            f"target=({target.x:.2f},{target.y:.2f},{math.degrees(target.yaw):.1f}deg) "
            f"raw_err=({raw_xy_error:.3f}m,{math.degrees(raw_yaw_error):.1f}deg) "
            f"active_err=({active_xy_error:.3f}m,{math.degrees(active_yaw_error):.1f}deg) "
            f"cmd=[{cmd.linear.x:.2f},{cmd.linear.y:.2f},{cmd.angular.z:.2f}]"
        )


def main() -> None:
    rclpy.init()
    node = FutureOdomPoseTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
