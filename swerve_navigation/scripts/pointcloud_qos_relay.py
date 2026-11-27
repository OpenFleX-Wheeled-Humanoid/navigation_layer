#!/usr/bin/env python3

import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2, PointField


PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _find_field_offset(fields, name):
    for f in fields:
        if f.name == name:
            return f.offset
    return None


class PointCloudQoSRelay(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_qos_relay')
        self.declare_parameter('input_topic', '/camera/d435/depth/color/points')
        self.declare_parameter('output_topic', '/camera/d435/depth/color/points_nav2')
        self.declare_parameter('restamp_on_receive', False)
        # Z-axis filtering in the point cloud's own frame (e.g. base_link)
        self.declare_parameter('filter_min_z', float('-inf'))
        self.declare_parameter('filter_max_z', float('inf'))
        # Self-box filtering: remove points inside a 3D box (robot body)
        self.declare_parameter('self_filter_enabled', False)
        self.declare_parameter('self_filter_min_x', -0.42)
        self.declare_parameter('self_filter_max_x', 0.42)
        self.declare_parameter('self_filter_min_y', -0.34)
        self.declare_parameter('self_filter_max_y', 0.34)
        self.declare_parameter('self_filter_min_z', -0.30)
        self.declare_parameter('self_filter_max_z', 0.45)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.restamp_on_receive = bool(self.get_parameter('restamp_on_receive').value)
        self.filter_min_z = float(self.get_parameter('filter_min_z').value)
        self.filter_max_z = float(self.get_parameter('filter_max_z').value)
        self.self_filter_enabled = bool(self.get_parameter('self_filter_enabled').value)
        self.self_filter_min_x = float(self.get_parameter('self_filter_min_x').value)
        self.self_filter_max_x = float(self.get_parameter('self_filter_max_x').value)
        self.self_filter_min_y = float(self.get_parameter('self_filter_min_y').value)
        self.self_filter_max_y = float(self.get_parameter('self_filter_max_y').value)
        self.self_filter_min_z = float(self.get_parameter('self_filter_min_z').value)
        self.self_filter_max_z = float(self.get_parameter('self_filter_max_z').value)

        self.filtering_enabled = (
            self.filter_min_z != float('-inf')
            or self.filter_max_z != float('inf')
            or self.self_filter_enabled
        )

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, PUB_QOS)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._callback,
            qos_profile_sensor_data,
        )
        self.forwarded = 0

        filter_info = ''
        if self.filter_min_z != float('-inf') or self.filter_max_z != float('inf'):
            filter_info += f', z_filter=[{self.filter_min_z:.2f}, {self.filter_max_z:.2f}]'
        if self.self_filter_enabled:
            filter_info += (
                f', self_filter=x[{self.self_filter_min_x:.2f},{self.self_filter_max_x:.2f}]'
                f' y[{self.self_filter_min_y:.2f},{self.self_filter_max_y:.2f}]'
                f' z[{self.self_filter_min_z:.2f},{self.self_filter_max_z:.2f}]'
            )
        self.get_logger().info(
            f'Relaying {self.input_topic} -> {self.output_topic} with RELIABLE QoS '
            f'(restamp_on_receive={self.restamp_on_receive}{filter_info})'
        )

    def _filter_cloud(self, msg: PointCloud2) -> PointCloud2:
        """Filter point cloud by Z range and/or self-box removal."""
        x_off = _find_field_offset(msg.fields, 'x')
        y_off = _find_field_offset(msg.fields, 'y')
        z_off = _find_field_offset(msg.fields, 'z')
        if z_off is None:
            return msg

        point_step = msg.point_step
        data = bytes(msg.data)
        n_points = msg.width * msg.height

        # Parse xyz as a structured array for fast numpy filtering
        xyz = np.ndarray(n_points, dtype=np.float32,
                         buffer=data, offset=z_off,
                         strides=(point_step,))
        mask = (xyz >= self.filter_min_z) & (xyz <= self.filter_max_z)

        if self.self_filter_enabled and x_off is not None and y_off is not None:
            xs = np.ndarray(n_points, dtype=np.float32,
                            buffer=data, offset=x_off, strides=(point_step,))
            ys = np.ndarray(n_points, dtype=np.float32,
                            buffer=data, offset=y_off, strides=(point_step,))
            in_box = (
                (xs >= self.self_filter_min_x) & (xs <= self.self_filter_max_x)
                & (ys >= self.self_filter_min_y) & (ys <= self.self_filter_max_y)
                & (xyz >= self.self_filter_min_z) & (xyz <= self.self_filter_max_z)
            )
            mask = mask & ~in_box

        if mask.all():
            return msg

        # Build filtered message
        indices = np.where(mask)[0]
        raw = np.frombuffer(data, dtype=np.uint8).reshape(n_points, point_step)
        filtered_data = raw[indices].tobytes()

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = len(indices)
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = out.point_step * out.width
        out.data = filtered_data
        out.is_dense = True
        return out

    def _callback(self, msg: PointCloud2) -> None:
        if self.restamp_on_receive:
            msg.header.stamp = self.get_clock().now().to_msg()
        if self.filtering_enabled:
            msg = self._filter_cloud(msg)
        self.publisher.publish(msg)
        self.forwarded += 1
        if self.forwarded == 1:
            self.get_logger().info(
                f'First point cloud relayed on {self.output_topic} '
                f'(stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d})'
            )


def main() -> None:
    rclpy.init()
    node = PointCloudQoSRelay()
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
