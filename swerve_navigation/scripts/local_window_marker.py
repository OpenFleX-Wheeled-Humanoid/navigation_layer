#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class LocalWindowMarker(Node):
    def __init__(self) -> None:
        super().__init__('local_window_marker')
        self.declare_parameter('topic', '/local_window_marker')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('size_x', 6.0)
        self.declare_parameter('size_y', 6.0)
        self.declare_parameter('thickness', 0.01)
        self.declare_parameter('z_offset', 0.005)
        self.declare_parameter('alpha', 0.18)
        self.declare_parameter('publish_period', 0.2)

        self.topic = str(self.get_parameter('topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.size_x = float(self.get_parameter('size_x').value)
        self.size_y = float(self.get_parameter('size_y').value)
        self.thickness = float(self.get_parameter('thickness').value)
        self.z_offset = float(self.get_parameter('z_offset').value)
        self.alpha = float(self.get_parameter('alpha').value)
        publish_period = float(self.get_parameter('publish_period').value)

        self.publisher = self.create_publisher(MarkerArray, self.topic, 1)
        self.timer = self.create_timer(publish_period, self._publish)
        self._publish()

    def _publish(self) -> None:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'local_window'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = self.z_offset
        marker.scale.x = self.size_x
        marker.scale.y = self.size_y
        marker.scale.z = self.thickness
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = self.alpha
        marker.lifetime = Duration(seconds=0.0).to_msg()

        array = MarkerArray()
        array.markers.append(marker)
        self.publisher.publish(array)


def main() -> None:
    rclpy.init()
    node = LocalWindowMarker()
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
