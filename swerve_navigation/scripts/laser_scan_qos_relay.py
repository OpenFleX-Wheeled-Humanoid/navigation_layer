#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan


PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class LaserScanQoSRelay(Node):
    def __init__(self) -> None:
        super().__init__('laser_scan_qos_relay')
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('restamp_on_receive', True)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.restamp_on_receive = bool(self.get_parameter('restamp_on_receive').value)

        self.publisher = self.create_publisher(LaserScan, self.output_topic, PUB_QOS)
        self.subscription = self.create_subscription(
            LaserScan,
            self.input_topic,
            self._callback,
            qos_profile_sensor_data,
        )
        self.forwarded = 0

        self.get_logger().info(
            f'Relaying {self.input_topic} -> {self.output_topic} with BEST_EFFORT QoS (depth=1) '
            f'(restamp_on_receive={self.restamp_on_receive})'
        )

    def _callback(self, msg: LaserScan) -> None:
        relay_msg = copy.copy(msg)
        if self.restamp_on_receive:
            relay_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(relay_msg)
        self.forwarded += 1
        if self.forwarded == 1:
            self.get_logger().info(
                f'First scan relayed on {self.output_topic} '
                f'(stamp={relay_msg.header.stamp.sec}.{relay_msg.header.stamp.nanosec:09d}, '
                f'frame={relay_msg.header.frame_id})'
            )


def main() -> None:
    rclpy.init()
    node = LaserScanQoSRelay()
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
