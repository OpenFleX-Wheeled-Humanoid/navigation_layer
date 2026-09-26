#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


@dataclass
class PlanarPose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def pose_from_odom(msg: Odometry) -> PlanarPose:
    return PlanarPose(
        x=msg.pose.pose.position.x,
        y=msg.pose.pose.position.y,
        z=msg.pose.pose.position.z,
        yaw=yaw_from_quaternion(msg.pose.pose.orientation),
    )


def compose_pose(a: PlanarPose, b: PlanarPose) -> PlanarPose:
    ca = math.cos(a.yaw)
    sa = math.sin(a.yaw)
    return PlanarPose(
        x=a.x + ca * b.x - sa * b.y,
        y=a.y + sa * b.x + ca * b.y,
        z=a.z + b.z,
        yaw=normalize_angle(a.yaw + b.yaw),
    )


def inverse_pose(pose: PlanarPose) -> PlanarPose:
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    return PlanarPose(
        x=-(c * pose.x + s * pose.y),
        y=-(-s * pose.x + c * pose.y),
        z=-pose.z,
        yaw=normalize_angle(-pose.yaw),
    )


def relative_pose(a: PlanarPose, b: PlanarPose) -> PlanarPose:
    return compose_pose(inverse_pose(a), b)


def interpolate_pose(a: PlanarPose, b: PlanarPose, alpha: float) -> PlanarPose:
    alpha = min(max(alpha, 0.0), 1.0)
    yaw_delta = normalize_angle(b.yaw - a.yaw)
    return PlanarPose(
        x=a.x + alpha * (b.x - a.x),
        y=a.y + alpha * (b.y - a.y),
        z=a.z + alpha * (b.z - a.z),
        yaw=normalize_angle(a.yaw + alpha * yaw_delta),
    )


def pose_distance_xy(pose: PlanarPose) -> float:
    return math.hypot(pose.x, pose.y)


class SafeOdomMux(Node):
    def __init__(self) -> None:
        super().__init__('safe_odom_mux')

        self.declare_parameter('lio_odom_topic', '/fastlio2/lio_odom')
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('output_odom_topic', '/odom_safe')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('lio_timeout', 0.30)
        self.declare_parameter('wheel_timeout', 0.30)
        self.declare_parameter('jump_pos_thresh', 0.30)
        self.declare_parameter('jump_yaw_thresh', 0.35)
        self.declare_parameter('motion_pos_thresh', 0.35)
        self.declare_parameter('motion_yaw_thresh', 0.35)
        self.declare_parameter('offset_alpha', 0.20)
        self.declare_parameter('recover_motion_pos_thresh', 0.08)
        self.declare_parameter('recover_motion_yaw_thresh', 0.10)
        self.declare_parameter('recover_offset_pos_thresh', 0.08)
        self.declare_parameter('recover_offset_yaw_thresh', 0.10)
        self.declare_parameter('recover_samples', 8)
        self.declare_parameter('recover_blend_alpha', 0.20)
        self.declare_parameter('recover_sync_pos_thresh', 0.03)
        self.declare_parameter('recover_sync_yaw_thresh', 0.04)
        self.declare_parameter('recover_sync_samples', 5)
        self.declare_parameter('wheel_delta_pos_thresh', 0.20)
        self.declare_parameter('wheel_delta_yaw_thresh', 0.35)

        self.lio_odom_topic = str(self.get_parameter('lio_odom_topic').value)
        self.wheel_odom_topic = str(self.get_parameter('wheel_odom_topic').value)
        self.output_odom_topic = str(self.get_parameter('output_odom_topic').value)
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.lio_timeout = float(self.get_parameter('lio_timeout').value)
        self.wheel_timeout = float(self.get_parameter('wheel_timeout').value)
        self.jump_pos_thresh = float(self.get_parameter('jump_pos_thresh').value)
        self.jump_yaw_thresh = float(self.get_parameter('jump_yaw_thresh').value)
        self.motion_pos_thresh = float(self.get_parameter('motion_pos_thresh').value)
        self.motion_yaw_thresh = float(self.get_parameter('motion_yaw_thresh').value)
        self.offset_alpha = float(self.get_parameter('offset_alpha').value)
        self.recover_motion_pos_thresh = float(
            self.get_parameter('recover_motion_pos_thresh').value)
        self.recover_motion_yaw_thresh = float(
            self.get_parameter('recover_motion_yaw_thresh').value)
        self.recover_offset_pos_thresh = float(
            self.get_parameter('recover_offset_pos_thresh').value)
        self.recover_offset_yaw_thresh = float(
            self.get_parameter('recover_offset_yaw_thresh').value)
        self.recover_samples = int(self.get_parameter('recover_samples').value)
        self.recover_blend_alpha = float(self.get_parameter('recover_blend_alpha').value)
        self.recover_sync_pos_thresh = float(
            self.get_parameter('recover_sync_pos_thresh').value)
        self.recover_sync_yaw_thresh = float(
            self.get_parameter('recover_sync_yaw_thresh').value)
        self.recover_sync_samples = int(self.get_parameter('recover_sync_samples').value)
        self.wheel_delta_pos_thresh = float(
            self.get_parameter('wheel_delta_pos_thresh').value)
        self.wheel_delta_yaw_thresh = float(
            self.get_parameter('wheel_delta_yaw_thresh').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, qos)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.lio_sub = self.create_subscription(
            Odometry, self.lio_odom_topic, self._on_lio_odom, qos)
        self.wheel_sub = self.create_subscription(
            Odometry, self.wheel_odom_topic, self._on_wheel_odom, qos)

        self.mode = 'INIT'
        self.lio_msg: Optional[Odometry] = None
        self.wheel_msg: Optional[Odometry] = None
        self.lio_pose: Optional[PlanarPose] = None
        self.wheel_pose: Optional[PlanarPose] = None
        self.prev_lio_pose: Optional[PlanarPose] = None
        self.prev_wheel_pose: Optional[PlanarPose] = None
        self.lio_rx_time = None
        self.wheel_rx_time = None
        self.offset_baseline: Optional[PlanarPose] = None
        self.output_anchor: Optional[PlanarPose] = None
        self.current_output_pose: Optional[PlanarPose] = None
        self.tracked_wheel_pose: Optional[PlanarPose] = None
        self.recovery_offset_prev: Optional[PlanarPose] = None
        self.recover_good_count = 0
        self.recover_sync_count = 0
        self.last_warn_reason = ''

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'Safe odom mux started: lio={self.lio_odom_topic}, '
            f'wheel={self.wheel_odom_topic}, output={self.output_odom_topic}'
        )

    def _on_lio_odom(self, msg: Odometry) -> None:
        if msg.child_frame_id and msg.child_frame_id != self.base_frame_id:
            return
        self.prev_lio_pose = self.lio_pose
        self.lio_msg = msg
        self.lio_pose = pose_from_odom(msg)
        self.lio_rx_time = self.get_clock().now()

    def _on_wheel_odom(self, msg: Odometry) -> None:
        if msg.child_frame_id and msg.child_frame_id != self.base_frame_id:
            return
        self.prev_wheel_pose = self.wheel_pose
        self.wheel_msg = msg
        self.wheel_pose = pose_from_odom(msg)
        self.wheel_rx_time = self.get_clock().now()

    def _is_fresh(self, rx_time, timeout: float) -> bool:
        if rx_time is None:
            return False
        return (self.get_clock().now() - rx_time).nanoseconds * 1e-9 <= timeout

    def _current_offset(self) -> Optional[PlanarPose]:
        if self.lio_pose is None or self.wheel_pose is None:
            return None
        return compose_pose(self.lio_pose, inverse_pose(self.wheel_pose))

    def _detect_lio_jump(self, current_offset: PlanarPose) -> Optional[str]:
        reasons = []
        if self.offset_baseline is not None:
            offset_delta = relative_pose(self.offset_baseline, current_offset)
            if pose_distance_xy(offset_delta) > self.jump_pos_thresh:
                reasons.append(
                    f'offset_pos={pose_distance_xy(offset_delta):.3f}m>{self.jump_pos_thresh:.3f}m')
            if abs(offset_delta.yaw) > self.jump_yaw_thresh:
                reasons.append(
                    f'offset_yaw={abs(offset_delta.yaw):.3f}rad>{self.jump_yaw_thresh:.3f}rad')

        if self.prev_lio_pose is not None and self.prev_wheel_pose is not None:
            lio_delta = relative_pose(self.prev_lio_pose, self.lio_pose)
            wheel_delta = relative_pose(self.prev_wheel_pose, self.wheel_pose)
            motion_err = relative_pose(wheel_delta, lio_delta)
            if pose_distance_xy(motion_err) > self.motion_pos_thresh:
                reasons.append(
                    f'motion_pos={pose_distance_xy(motion_err):.3f}m>{self.motion_pos_thresh:.3f}m')
            if abs(motion_err.yaw) > self.motion_yaw_thresh:
                reasons.append(
                    f'motion_yaw={abs(motion_err.yaw):.3f}rad>{self.motion_yaw_thresh:.3f}rad')

        if not reasons:
            return None
        return ', '.join(reasons)

    def _recovery_looks_healthy(self, current_offset: PlanarPose) -> bool:
        if self.prev_lio_pose is None or self.prev_wheel_pose is None:
            self.recovery_offset_prev = current_offset
            return False

        healthy = True
        lio_delta = relative_pose(self.prev_lio_pose, self.lio_pose)
        wheel_delta = relative_pose(self.prev_wheel_pose, self.wheel_pose)
        motion_err = relative_pose(wheel_delta, lio_delta)

        if pose_distance_xy(motion_err) > self.recover_motion_pos_thresh:
            healthy = False
        if abs(motion_err.yaw) > self.recover_motion_yaw_thresh:
            healthy = False

        if self.recovery_offset_prev is not None:
            offset_step = relative_pose(self.recovery_offset_prev, current_offset)
            if pose_distance_xy(offset_step) > self.recover_offset_pos_thresh:
                healthy = False
            if abs(offset_step.yaw) > self.recover_offset_yaw_thresh:
                healthy = False

        self.recovery_offset_prev = current_offset
        return healthy

    def _enter_wheel_mode(self, reason: str) -> None:
        if self.wheel_pose is None:
            return
        if self.current_output_pose is None:
            self.current_output_pose = self.wheel_pose
        self.tracked_wheel_pose = self.wheel_pose
        self.output_anchor = None

        if self.mode != 'WHEEL' or reason != self.last_warn_reason:
            self.get_logger().warn(
                f'Switching odom source to wheel odom, reason: {reason}')
        self.mode = 'WHEEL'
        self.last_warn_reason = reason
        self.recover_good_count = 0
        self.recover_sync_count = 0
        self.recovery_offset_prev = None

    def _switch_to_lio(self, reason: str) -> None:
        was_lio = self.mode == 'LIO'
        self.mode = 'LIO'
        self.last_warn_reason = ''
        self.recover_good_count = 0
        self.recover_sync_count = 0
        self.recovery_offset_prev = None
        self.tracked_wheel_pose = self.wheel_pose
        if not was_lio:
            self.get_logger().info(f'Switching odom source back to FAST-LIO, reason: {reason}')

    def _advance_with_wheel_delta(self) -> Optional[str]:
        if self.wheel_pose is None:
            return 'wheel pose unavailable'

        if self.current_output_pose is None:
            self.current_output_pose = self.wheel_pose
            self.tracked_wheel_pose = self.wheel_pose
            return None

        if self.tracked_wheel_pose is None:
            self.tracked_wheel_pose = self.wheel_pose
            return None

        wheel_delta = relative_pose(self.tracked_wheel_pose, self.wheel_pose)
        delta_pos = pose_distance_xy(wheel_delta)
        delta_yaw = abs(wheel_delta.yaw)
        if delta_pos > self.wheel_delta_pos_thresh:
            return f'wheel_delta_pos={delta_pos:.3f}m>{self.wheel_delta_pos_thresh:.3f}m'
        if delta_yaw > self.wheel_delta_yaw_thresh:
            return f'wheel_delta_yaw={delta_yaw:.3f}rad>{self.wheel_delta_yaw_thresh:.3f}rad'

        self.current_output_pose = compose_pose(self.current_output_pose, wheel_delta)
        self.tracked_wheel_pose = self.wheel_pose
        return None

    def _publish_output(self, pose: PlanarPose, use_lio_pose: bool) -> None:
        now_msg = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp = now_msg
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.position.z = pose.z
        qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        pose_source = self.lio_msg if use_lio_pose and self.lio_msg is not None else self.wheel_msg
        if pose_source is not None:
            msg.pose.covariance = list(pose_source.pose.covariance)

        twist_source = self.wheel_msg if self.wheel_msg is not None else self.lio_msg
        if twist_source is not None:
            msg.twist = twist_source.twist

        self.odom_pub.publish(msg)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now_msg
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = pose.x
            tf_msg.transform.translation.y = pose.y
            tf_msg.transform.translation.z = pose.z
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

        self.current_output_pose = pose

    def _on_timer(self) -> None:
        lio_fresh = self._is_fresh(self.lio_rx_time, self.lio_timeout)
        wheel_fresh = self._is_fresh(self.wheel_rx_time, self.wheel_timeout)

        if not lio_fresh and not wheel_fresh:
            return

        if wheel_fresh and self.wheel_pose is None:
            return
        if lio_fresh and self.lio_pose is None:
            return

        if not wheel_fresh:
            if self.lio_pose is not None:
                self.offset_baseline = None
                self.output_anchor = None
                self._switch_to_lio('wheel odom unavailable')
                self.current_output_pose = self.lio_pose
                self._publish_output(self.lio_pose, use_lio_pose=True)
            return

        if not lio_fresh:
            self._enter_wheel_mode('FAST-LIO timeout')
            wheel_reject_reason = self._advance_with_wheel_delta()
            if wheel_reject_reason is not None:
                self.get_logger().warn(
                    f'Wheel odom delta rejected, freezing safe odom: {wheel_reject_reason}')
            if self.current_output_pose is not None:
                self._publish_output(self.current_output_pose, use_lio_pose=False)
            return

        current_offset = self._current_offset()
        if current_offset is None:
            return

        if self.mode in ('INIT', 'LIO'):
            jump_reason = self._detect_lio_jump(current_offset)
            if jump_reason is None:
                if self.offset_baseline is None:
                    self.offset_baseline = current_offset
                else:
                    self.offset_baseline = interpolate_pose(
                        self.offset_baseline, current_offset, self.offset_alpha)
                self.output_anchor = current_offset
                self.mode = 'LIO'
                self.current_output_pose = self.lio_pose
                self.tracked_wheel_pose = self.wheel_pose
                self._publish_output(self.lio_pose, use_lio_pose=True)
                return

            self._enter_wheel_mode(jump_reason)

        if self.mode == 'WHEEL':
            wheel_reject_reason = self._advance_with_wheel_delta()
            if wheel_reject_reason is not None:
                self.get_logger().warn(
                    f'Wheel odom delta rejected, freezing safe odom: {wheel_reject_reason}')
            if self.current_output_pose is not None:
                self._publish_output(self.current_output_pose, use_lio_pose=False)

            if self._recovery_looks_healthy(current_offset):
                self.recover_good_count += 1
                if self.recover_good_count >= self.recover_samples:
                    self.mode = 'RECOVERING'
                    self.recover_sync_count = 0
                    self.get_logger().info('FAST-LIO looks stable again, starting smooth recovery')
            else:
                self.recover_good_count = 0
            return

        if self.mode == 'RECOVERING':
            if not self._recovery_looks_healthy(current_offset):
                self.mode = 'WHEEL'
                self.recover_good_count = 0
                self.recover_sync_count = 0
                if self.current_output_pose is not None:
                    self._publish_output(self.current_output_pose, use_lio_pose=False)
                return

            wheel_reject_reason = self._advance_with_wheel_delta()
            if wheel_reject_reason is not None:
                self.get_logger().warn(
                    f'Wheel odom delta rejected during recovery, freezing safe odom: {wheel_reject_reason}')

            if self.current_output_pose is None:
                return

            self.current_output_pose = interpolate_pose(
                self.current_output_pose, self.lio_pose, self.recover_blend_alpha)
            self._publish_output(self.current_output_pose, use_lio_pose=False)

            sync_error = relative_pose(self.current_output_pose, self.lio_pose)
            if (pose_distance_xy(sync_error) <= self.recover_sync_pos_thresh and
                    abs(sync_error.yaw) <= self.recover_sync_yaw_thresh):
                self.recover_sync_count += 1
            else:
                self.recover_sync_count = 0

            if self.recover_sync_count >= self.recover_sync_samples:
                self.offset_baseline = current_offset
                self._switch_to_lio('recovery anchor converged')
                self.current_output_pose = self.lio_pose
                self._publish_output(self.lio_pose, use_lio_pose=True)


def main() -> None:
    rclpy.init()
    node = SafeOdomMux()
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
