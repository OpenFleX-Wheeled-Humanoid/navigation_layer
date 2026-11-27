#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Literal

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


Mode = Literal['idle', 'straight', 'rotate', 'normal']


class CmdVelReplayConditioner(Node):
    """Condition replay cmd_vel before velocity smoothing.

    VLA replay short-horizon paths can produce small lateral / yaw-rate
    residuals even during visually straight motion. Those residuals are enough
    to make a swerve chassis continually retarget steering angles. This node
    applies component deadbands plus simple hysteretic motion-mode locks before
    the Nav2 velocity smoother.
    """

    def __init__(self) -> None:
        super().__init__('cmd_vel_replay_conditioner')

        self.declare_parameter('input_cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('output_cmd_topic', '/cmd_vel_conditioned')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('cmd_timeout', 0.35)

        self.declare_parameter('linear_x_deadband', 0.01)
        self.declare_parameter('linear_y_deadband', 0.025)
        self.declare_parameter('angular_z_deadband', 0.035)

        self.declare_parameter('straight_lock_enabled', True)
        self.declare_parameter('straight_enter_min_abs_vx', 0.08)
        self.declare_parameter('straight_enter_max_abs_vy', 0.035)
        self.declare_parameter('straight_enter_max_abs_wz', 0.04)
        self.declare_parameter('straight_exit_min_abs_vx', 0.05)
        self.declare_parameter('straight_exit_max_abs_vy', 0.055)
        self.declare_parameter('straight_exit_max_abs_wz', 0.065)

        self.declare_parameter('rotate_lock_enabled', True)
        self.declare_parameter('rotate_enter_min_abs_wz', 0.08)
        self.declare_parameter('rotate_enter_max_linear_speed', 0.04)
        self.declare_parameter('rotate_exit_min_abs_wz', 0.055)
        self.declare_parameter('rotate_exit_max_linear_speed', 0.07)

        self.declare_parameter('ema_alpha', 1.0)

        self.input_cmd_topic = str(self.get_parameter('input_cmd_topic').value)
        self.output_cmd_topic = str(self.get_parameter('output_cmd_topic').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.linear_x_deadband = float(self.get_parameter('linear_x_deadband').value)
        self.linear_y_deadband = float(self.get_parameter('linear_y_deadband').value)
        self.angular_z_deadband = float(self.get_parameter('angular_z_deadband').value)

        self.straight_lock_enabled = bool(
            self.get_parameter('straight_lock_enabled').value)
        self.straight_enter_min_abs_vx = float(
            self.get_parameter('straight_enter_min_abs_vx').value)
        self.straight_enter_max_abs_vy = float(
            self.get_parameter('straight_enter_max_abs_vy').value)
        self.straight_enter_max_abs_wz = float(
            self.get_parameter('straight_enter_max_abs_wz').value)
        self.straight_exit_min_abs_vx = float(
            self.get_parameter('straight_exit_min_abs_vx').value)
        self.straight_exit_max_abs_vy = float(
            self.get_parameter('straight_exit_max_abs_vy').value)
        self.straight_exit_max_abs_wz = float(
            self.get_parameter('straight_exit_max_abs_wz').value)

        self.rotate_lock_enabled = bool(self.get_parameter('rotate_lock_enabled').value)
        self.rotate_enter_min_abs_wz = float(
            self.get_parameter('rotate_enter_min_abs_wz').value)
        self.rotate_enter_max_linear_speed = float(
            self.get_parameter('rotate_enter_max_linear_speed').value)
        self.rotate_exit_min_abs_wz = float(
            self.get_parameter('rotate_exit_min_abs_wz').value)
        self.rotate_exit_max_linear_speed = float(
            self.get_parameter('rotate_exit_max_linear_speed').value)

        self.ema_alpha = max(0.0, min(1.0, float(self.get_parameter('ema_alpha').value)))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.pub = self.create_publisher(Twist, self.output_cmd_topic, qos)
        self.create_subscription(Twist, self.input_cmd_topic, self._on_cmd, qos)

        self.mode: Mode = 'idle'
        self.last_conditioned = Twist()
        self.last_raw_rx_time = None

        period = 1.0 / max(self.publish_rate, 1.0)
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'cmd_vel replay conditioner started: in={self.input_cmd_topic}, '
            f'out={self.output_cmd_topic}, rate={self.publish_rate:.1f}Hz'
        )

    @staticmethod
    def _copy_twist(msg: Twist) -> Twist:
        out = Twist()
        out.linear.x = float(msg.linear.x)
        out.linear.y = float(msg.linear.y)
        out.linear.z = float(msg.linear.z)
        out.angular.x = float(msg.angular.x)
        out.angular.y = float(msg.angular.y)
        out.angular.z = float(msg.angular.z)
        return out

    @staticmethod
    def _apply_deadband(value: float, threshold: float) -> float:
        return 0.0 if abs(value) < max(0.0, threshold) else value

    @staticmethod
    def _linear_speed(msg: Twist) -> float:
        return math.hypot(float(msg.linear.x), float(msg.linear.y))

    def _on_cmd(self, msg: Twist) -> None:
        self.last_raw_rx_time = self.get_clock().now()
        conditioned = self._condition(msg)

        if self.ema_alpha < 1.0:
            alpha = self.ema_alpha
            conditioned.linear.x = (
                alpha * conditioned.linear.x + (1.0 - alpha) * self.last_conditioned.linear.x)
            conditioned.linear.y = (
                alpha * conditioned.linear.y + (1.0 - alpha) * self.last_conditioned.linear.y)
            conditioned.angular.z = (
                alpha * conditioned.angular.z + (1.0 - alpha) * self.last_conditioned.angular.z)

        self.last_conditioned = conditioned
        self.pub.publish(conditioned)

    def _on_timer(self) -> None:
        if self.last_raw_rx_time is None:
            return

        age = (self.get_clock().now() - self.last_raw_rx_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            if self.mode != 'idle':
                self.get_logger().info(
                    f'replay cmd timeout ({age:.3f}s), forcing zero velocity')
            self.mode = 'idle'
            zero = Twist()
            self.last_conditioned = zero
            self.pub.publish(zero)

    def _condition(self, msg: Twist) -> Twist:
        out = self._copy_twist(msg)
        out.linear.x = self._apply_deadband(out.linear.x, self.linear_x_deadband)
        out.linear.y = self._apply_deadband(out.linear.y, self.linear_y_deadband)
        out.angular.z = self._apply_deadband(out.angular.z, self.angular_z_deadband)

        abs_vx = abs(out.linear.x)
        abs_vy = abs(out.linear.y)
        abs_wz = abs(out.angular.z)
        linear_speed = self._linear_speed(out)

        if self.mode == 'straight':
            stay_straight = (
                abs_vx >= self.straight_exit_min_abs_vx
                and abs_vy <= self.straight_exit_max_abs_vy
                and abs_wz <= self.straight_exit_max_abs_wz
            )
            if not stay_straight:
                self.mode = 'normal'
        elif self.mode == 'rotate':
            stay_rotate = (
                abs_wz >= self.rotate_exit_min_abs_wz
                and linear_speed <= self.rotate_exit_max_linear_speed
            )
            if not stay_rotate:
                self.mode = 'normal'

        if self.mode not in ('straight', 'rotate'):
            if (
                self.straight_lock_enabled
                and
                abs_vx >= self.straight_enter_min_abs_vx
                and abs_vy <= self.straight_enter_max_abs_vy
                and abs_wz <= self.straight_enter_max_abs_wz
            ):
                self.mode = 'straight'
            elif (
                self.rotate_lock_enabled
                and
                abs_wz >= self.rotate_enter_min_abs_wz
                and linear_speed <= self.rotate_enter_max_linear_speed
            ):
                self.mode = 'rotate'
            elif linear_speed < self.linear_x_deadband and abs_wz < self.angular_z_deadband:
                self.mode = 'idle'
            else:
                self.mode = 'normal'

        if self.mode == 'straight':
            out.linear.y = 0.0
            out.angular.z = 0.0
        elif self.mode == 'rotate':
            out.linear.x = 0.0
            out.linear.y = 0.0
        elif self.mode == 'idle':
            out = Twist()

        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        return out


def main() -> None:
    rclpy.init()
    node = CmdVelReplayConditioner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
