#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty


class CmdVelSafetyGate(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_safety_gate')

        self.declare_parameter('input_cmd_topic', '/cmd_vel_safe_in')
        self.declare_parameter('output_cmd_topic', '/cmd_vel_safe')
        self.declare_parameter('hold_topic', '/localization_hold')
        self.declare_parameter('hold_duration', 1.5)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('publish_rate', 5.0)

        self.input_cmd_topic = str(self.get_parameter('input_cmd_topic').value)
        self.output_cmd_topic = str(self.get_parameter('output_cmd_topic').value)
        self.hold_topic = str(self.get_parameter('hold_topic').value)
        self.hold_duration = float(self.get_parameter('hold_duration').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, qos)
        self.create_subscription(Twist, self.input_cmd_topic, self._on_cmd, qos)
        self.create_subscription(Empty, self.hold_topic, self._on_hold, qos)

        self.last_cmd: Optional[Twist] = None
        self.last_cmd_time = None
        self.hold_until = None
        self.hold_active = False

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'cmd_vel safety gate started: in={self.input_cmd_topic}, '
            f'out={self.output_cmd_topic}, hold={self.hold_topic}, '
            f'hold_duration={self.hold_duration:.2f}s'
        )

    def _on_cmd(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()
        if not self._hold_is_active():
            self.cmd_pub.publish(msg)

    def _on_hold(self, _msg: Empty) -> None:
        self.hold_until = self.get_clock().now() + Duration(seconds=self.hold_duration)
        if not self.hold_active:
            self.get_logger().warn(
                f'Localization hold received, forcing zero cmd_vel for {self.hold_duration:.2f}s')
        self.hold_active = True
        self.cmd_pub.publish(Twist())

    def _hold_is_active(self) -> bool:
        if self.hold_until is None:
            return False
        return self.get_clock().now() < self.hold_until

    def _cmd_is_fresh(self) -> bool:
        if self.last_cmd is None or self.last_cmd_time is None:
            return False
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        return age <= self.cmd_timeout

    def _on_timer(self) -> None:
        if self._hold_is_active():
            self.cmd_pub.publish(Twist())
            return

        if self.hold_active:
            self.hold_active = False
            self.get_logger().info('Localization hold cleared, resuming cmd_vel relay')

        if not self._cmd_is_fresh():
            self.cmd_pub.publish(Twist())


def main() -> None:
    rclpy.init()
    node = CmdVelSafetyGate()
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
