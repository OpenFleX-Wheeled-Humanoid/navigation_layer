#!/usr/bin/env python3
"""Bridge VLA base paths to Nav2 FollowPath goals."""

from __future__ import annotations

import math
import threading
from copy import deepcopy

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node


class VlaPathToNav2FollowPath(Node):
    """Send the latest VLA path to Nav2 controller_server FollowPath."""

    def __init__(self) -> None:
        super().__init__("vla_path_to_nav2_follow_path")

        self.declare_parameter("input_path_topic", "/vla/base_path")
        self.declare_parameter("odom_topic", "/fastlio2/lio_odom")
        self.declare_parameter("follow_path_action_name", "/follow_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")
        self.declare_parameter("min_goal_update_period", 0.15)
        self.declare_parameter("path_timeout", 0.5)
        self.declare_parameter("min_goal_delta_xy", 0.03)
        self.declare_parameter("min_goal_delta_yaw", 0.04)
        self.declare_parameter("stop_publish_count", 6)
        self.declare_parameter("prepend_current_pose", True)
        self.declare_parameter("min_path_points", 2)
        self.declare_parameter("strict_frame_check", True)
        self.declare_parameter("rotation_guard_enabled", True)
        self.declare_parameter("rotation_xy_span_threshold", 0.03)
        self.declare_parameter("rotation_min_yaw_delta", 0.10)
        self.declare_parameter("rotation_release_yaw_error", 0.087)
        self.declare_parameter("rotation_release_angular_speed", 0.12)
        self.declare_parameter("rotation_release_stable_samples", 3)
        self.declare_parameter("rotation_hold_path_points", 8)
        self.declare_parameter("rotation_hold_resend_period", 0.35)
        self.declare_parameter("rotation_max_hold_duration", 1.5)
        self.declare_parameter("rotation_timeout_release_yaw_error", 0.14)
        self.declare_parameter("static_lock_enabled", True)
        self.declare_parameter("static_lock_xy_tolerance", 0.07)
        self.declare_parameter("static_lock_yaw_tolerance", 0.06)
        self.declare_parameter("static_lock_entry_xy_tolerance", 0.10)
        self.declare_parameter("static_lock_entry_yaw_tolerance", 0.10)
        self.declare_parameter("static_lock_release_xy_tolerance", 0.10)
        self.declare_parameter("static_lock_release_yaw_tolerance", 0.10)
        self.declare_parameter("static_lock_linear_speed", 0.05)
        self.declare_parameter("static_lock_angular_speed", 0.06)
        self.declare_parameter("static_lock_stable_samples", 3)

        self.input_path_topic = str(self.get_parameter("input_path_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.follow_path_action_name = str(self.get_parameter("follow_path_action_name").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.controller_id = str(self.get_parameter("controller_id").value)
        self.goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        self.min_goal_update_period = float(self.get_parameter("min_goal_update_period").value)
        self.path_timeout = float(self.get_parameter("path_timeout").value)
        self.min_goal_delta_xy = float(self.get_parameter("min_goal_delta_xy").value)
        self.min_goal_delta_yaw = float(self.get_parameter("min_goal_delta_yaw").value)
        self.stop_publish_count = max(1, int(self.get_parameter("stop_publish_count").value))
        self.prepend_current_pose = bool(self.get_parameter("prepend_current_pose").value)
        self.min_path_points = max(1, int(self.get_parameter("min_path_points").value))
        self.strict_frame_check = bool(self.get_parameter("strict_frame_check").value)
        self.rotation_guard_enabled = bool(self.get_parameter("rotation_guard_enabled").value)
        self.rotation_xy_span_threshold = float(
            self.get_parameter("rotation_xy_span_threshold").value)
        self.rotation_min_yaw_delta = float(self.get_parameter("rotation_min_yaw_delta").value)
        self.rotation_release_yaw_error = float(
            self.get_parameter("rotation_release_yaw_error").value)
        self.rotation_release_angular_speed = float(
            self.get_parameter("rotation_release_angular_speed").value)
        self.rotation_release_stable_samples = max(
            1, int(self.get_parameter("rotation_release_stable_samples").value))
        self.rotation_hold_path_points = max(
            2, int(self.get_parameter("rotation_hold_path_points").value))
        self.rotation_hold_resend_period = max(
            0.0, float(self.get_parameter("rotation_hold_resend_period").value))
        self.rotation_max_hold_duration = max(
            0.0, float(self.get_parameter("rotation_max_hold_duration").value))
        self.rotation_timeout_release_yaw_error = max(
            self.rotation_release_yaw_error,
            float(self.get_parameter("rotation_timeout_release_yaw_error").value))
        self.static_lock_enabled = bool(self.get_parameter("static_lock_enabled").value)
        self.static_lock_xy_tolerance = max(
            0.0, float(self.get_parameter("static_lock_xy_tolerance").value))
        self.static_lock_yaw_tolerance = max(
            0.0, float(self.get_parameter("static_lock_yaw_tolerance").value))
        self.static_lock_entry_xy_tolerance = max(
            self.static_lock_xy_tolerance,
            float(self.get_parameter("static_lock_entry_xy_tolerance").value))
        self.static_lock_entry_yaw_tolerance = max(
            self.static_lock_yaw_tolerance,
            float(self.get_parameter("static_lock_entry_yaw_tolerance").value))
        self.static_lock_release_xy_tolerance = max(
            self.static_lock_xy_tolerance,
            float(self.get_parameter("static_lock_release_xy_tolerance").value))
        self.static_lock_release_yaw_tolerance = max(
            self.static_lock_yaw_tolerance,
            float(self.get_parameter("static_lock_release_yaw_tolerance").value))
        self.static_lock_linear_speed = max(
            0.0, float(self.get_parameter("static_lock_linear_speed").value))
        self.static_lock_angular_speed = max(
            0.0, float(self.get_parameter("static_lock_angular_speed").value))
        self.static_lock_stable_samples = max(
            1, int(self.get_parameter("static_lock_stable_samples").value))

        self._lock = threading.Lock()
        self._latest_odom: Odometry | None = None
        # Protected by _lock:
        self._last_goal_path: Path | None = None
        self._last_goal_time_s = 0.0
        self._last_path_rx_time_s = 0.0
        self._goal_handle = None
        self._goal_generation = 0
        self._goal_in_flight = False
        self._stopped_for_current_gap = False
        self._rotation_guard_active = False
        self._rotation_target_yaw = 0.0
        self._rotation_release_ready_count = 0
        self._rotation_hold_start_time_s = 0.0
        self._last_rotation_hold_goal_time_s = 0.0
        self._force_next_goal_update = False
        self._static_lock_active = False
        self._static_lock_ready_count = 0
        self._static_lock_goal_valid = False
        self._static_lock_goal_x = 0.0
        self._static_lock_goal_y = 0.0
        self._static_lock_goal_yaw = 0.0

        self._client = ActionClient(self, FollowPath, self.follow_path_action_name)
        self._cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Path, self.input_path_topic, self._path_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 50)
        self.create_timer(0.1, self._timeout_check)

        self.get_logger().info(
            f"VLA Path bridge ready: {self.input_path_topic} -> "
            f"{self.follow_path_action_name} controller_id={self.controller_id} "
            f"odom_topic={self.odom_topic}"
        )

    def destroy_node(self) -> None:
        """Clean up active goals before destroying the node."""
        self._stop_tracking("node shutdown", force=True)
        super().destroy_node()

    @staticmethod
    def _yaw_from_orientation(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @classmethod
    def _yaw_from_pose_stamped(cls, pose) -> float:
        return cls._yaw_from_orientation(pose.pose.orientation)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            self._latest_odom = msg

    def _path_cb(self, msg: Path) -> None:
        try:
            self._path_cb_impl(msg)
        except Exception as exc:
            self.get_logger().error(f"Exception in _path_cb: {exc}", throttle_duration_sec=2.0)

    def _path_cb_impl(self, msg: Path) -> None:
        with self._lock:
            self._last_path_rx_time_s = self._now_s()
            self._stopped_for_current_gap = False

        if not msg.poses:
            self._stop_tracking("empty VLA path", force=True)
            return

        if not self._path_frame_is_valid(msg):
            return

        now_s = self._now_s()
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            age_s = now_s - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            if age_s > self.path_timeout:
                self.get_logger().warning(
                    f"Dropping stale VLA path: age={age_s:.3f}s timeout={self.path_timeout:.3f}s"
                )
                return
            if age_s < -0.1:  # Future timestamp (clock skew tolerance: 100ms)
                self.get_logger().warning(
                    f"Dropping VLA path with future timestamp: age={age_s:.3f}s"
                )
                return

        path = self._prepare_guarded_path(msg)
        if len(path.poses) < self.min_path_points:
            return

        if self._update_static_lock(msg):
            return

        with self._lock:
            if not self._should_send_goal_locked(path, now_s):
                return

            if self._goal_in_flight:
                return

            self._goal_in_flight = True
            self._force_next_goal_update = False
            self._goal_generation += 1
            goal_generation = self._goal_generation

        if not self._client.server_is_ready() and not self._client.wait_for_server(timeout_sec=0.05):
            self.get_logger().warning(
                f"Nav2 FollowPath action server is not ready: {self.follow_path_action_name}"
            )
            with self._lock:
                self._goal_in_flight = False
            return

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future: self._goal_response_cb(future, path, now_s, goal_generation))

    def _path_frame_is_valid(self, msg: Path) -> bool:
        if not msg.poses:
            return False
        path_frame = msg.header.frame_id or msg.poses[0].header.frame_id
        with self._lock:
            odom = self._latest_odom
        odom_frame = odom.header.frame_id if odom is not None else ""

        if not path_frame:
            self.get_logger().warning("Dropping VLA path without a frame_id")
            return False

        if odom_frame and path_frame != odom_frame:
            message = (
                f"VLA path frame does not match odom frame: path={path_frame} "
                f"odom={odom_frame}. Keep robot.ros2.base_path_frame_id, "
                "teleop/follower odom_topic, and bridge odom_topic consistent."
            )
            if self.strict_frame_check:
                self.get_logger().error(message)
                return False
            self.get_logger().warning(message)

        return True

    def _path_first_yaw(self, msg: Path) -> float:
        return self._yaw_from_pose_stamped(msg.poses[0])

    def _path_last_yaw(self, msg: Path) -> float:
        return self._yaw_from_pose_stamped(msg.poses[-1])

    def _path_yaw_delta(self, msg: Path) -> float:
        return self._normalize_angle(self._path_last_yaw(msg) - self._path_first_yaw(msg))

    @staticmethod
    def _path_xy_span(msg: Path) -> float:
        if not msg.poses:
            return 0.0

        first = msg.poses[0].pose.position
        return max(
            math.hypot(
                pose.pose.position.x - first.x,
                pose.pose.position.y - first.y,
            )
            for pose in msg.poses
        )

    def _is_stationary_xy_path(self, msg: Path) -> bool:
        return self._path_xy_span(msg) <= self.rotation_xy_span_threshold

    def _is_pure_rotation_path(self, msg: Path) -> bool:
        if len(msg.poses) < 2:
            return False
        return (
            self._is_stationary_xy_path(msg)
            and abs(self._path_yaw_delta(msg)) >= self.rotation_min_yaw_delta
        )

    def _is_static_path(self, msg: Path) -> bool:
        if not msg.poses:
            return False
        return (
            self._is_stationary_xy_path(msg)
            and abs(self._path_yaw_delta(msg)) < self.rotation_min_yaw_delta
        )

    def _odom_error_to_path_goal(self, odom: Odometry, msg: Path) -> tuple[float, float]:
        goal = msg.poses[-1].pose
        odom_pose = odom.pose.pose
        dx = goal.position.x - odom_pose.position.x
        dy = goal.position.y - odom_pose.position.y
        xy_error = math.hypot(dx, dy)
        yaw_error = abs(
            self._normalize_angle(
                self._yaw_from_orientation(goal.orientation) - self._odom_yaw(odom)))
        return xy_error, yaw_error

    @staticmethod
    def _odom_linear_speed(odom: Odometry) -> float:
        twist = odom.twist.twist
        return math.hypot(float(twist.linear.x), float(twist.linear.y))

    def _reset_static_lock_state(self) -> None:
        with self._lock:
            was_active = self._static_lock_active
            self._static_lock_active = False
            self._static_lock_ready_count = 0
            self._static_lock_goal_valid = False
        if was_active:
            self.get_logger().info("Released VLA static lock")

    def _static_goal_changed_locked(self, msg: Path) -> bool:
        if not self._static_lock_goal_valid:
            return False
        goal = msg.poses[-1].pose
        dx = goal.position.x - self._static_lock_goal_x
        dy = goal.position.y - self._static_lock_goal_y
        dyaw = abs(
            self._normalize_angle(
                self._yaw_from_orientation(goal.orientation) - self._static_lock_goal_yaw))
        return (
            math.hypot(dx, dy) > self.static_lock_release_xy_tolerance
            or dyaw > self.static_lock_release_yaw_tolerance
        )

    def _capture_static_lock_goal_locked(self, msg: Path) -> None:
        goal = msg.poses[-1].pose
        self._static_lock_goal_x = goal.position.x
        self._static_lock_goal_y = goal.position.y
        self._static_lock_goal_yaw = self._yaw_from_orientation(goal.orientation)
        self._static_lock_goal_valid = True

    def _update_static_lock(self, msg: Path) -> bool:
        if not self.static_lock_enabled:
            return False

        is_static_path = self._is_static_path(msg)
        with self._lock:
            odom = self._latest_odom
            rotation_guard_active = self._rotation_guard_active
            static_lock_active = self._static_lock_active

        if not is_static_path:
            if static_lock_active:
                self._reset_static_lock_state()
            else:
                with self._lock:
                    self._static_lock_ready_count = 0
            return False

        if odom is None or rotation_guard_active:
            return static_lock_active

        xy_error, yaw_error = self._odom_error_to_path_goal(odom, msg)
        linear_speed = self._odom_linear_speed(odom)
        angular_speed = abs(float(odom.twist.twist.angular.z))
        if static_lock_active:
            with self._lock:
                goal_changed = self._static_goal_changed_locked(msg)
            if goal_changed:
                self._reset_static_lock_state()
                return False
            self._publish_zero_velocity()
            return True

        settling_ready = (
            xy_error <= self.static_lock_entry_xy_tolerance
            and yaw_error <= self.static_lock_entry_yaw_tolerance
            and linear_speed <= self.static_lock_linear_speed
            and angular_speed <= self.static_lock_angular_speed
        )
        stopped_ready = (
            xy_error <= self.static_lock_xy_tolerance
            and yaw_error <= self.static_lock_yaw_tolerance
            and linear_speed <= self.static_lock_linear_speed
            and angular_speed <= self.static_lock_angular_speed
        )
        ready = settling_ready or stopped_ready

        with self._lock:
            if ready:
                self._static_lock_ready_count += 1
            else:
                self._static_lock_ready_count = 0
            ready_count = self._static_lock_ready_count
            should_activate = ready_count >= self.static_lock_stable_samples
            if should_activate:
                self._static_lock_active = True
                self._last_goal_path = None
                self._last_goal_time_s = 0.0
                self._force_next_goal_update = False
                self._goal_generation += 1
                self._goal_in_flight = False
                self._capture_static_lock_goal_locked(msg)

        if should_activate:
            self.get_logger().info(
                "VLA static lock active: "
                f"xy_error={xy_error:.3f} yaw_error={math.degrees(yaw_error):.1f}deg "
                f"v={linear_speed:.3f} wz={angular_speed:.3f}"
            )
            self._publish_zero_velocity()
            self._cancel_active_goal("VLA static lock")
            return True

        return False

    def _odom_yaw(self, odom: Odometry) -> float:
        return self._yaw_from_orientation(odom.pose.pose.orientation)

    def _rotation_release_reached(self, odom: Odometry, target_yaw: float) -> bool:
        yaw_error = abs(self._normalize_angle(target_yaw - self._odom_yaw(odom)))
        angular_speed = abs(float(odom.twist.twist.angular.z))
        return (
            yaw_error <= self.rotation_release_yaw_error
            and angular_speed <= self.rotation_release_angular_speed
        )

    def _make_rotation_hold_path(self, template: Path, odom: Odometry, target_yaw: float) -> Path:
        path = Path()
        path.header = template.header
        frame_id = template.header.frame_id or odom.header.frame_id
        path.header.frame_id = frame_id

        start_yaw = self._odom_yaw(odom)
        yaw_delta = self._normalize_angle(target_yaw - start_yaw)
        count = max(self.rotation_hold_path_points, self.min_path_points)
        base_pose = template.poses[0]

        for i in range(count):
            pose = deepcopy(base_pose)
            ratio = i / (count - 1) if count > 1 else 1.0
            yaw = start_yaw + yaw_delta * ratio
            pose.header.stamp = template.header.stamp
            pose.header.frame_id = frame_id
            pose.pose.position.x = odom.pose.pose.position.x
            pose.pose.position.y = odom.pose.pose.position.y
            pose.pose.position.z = odom.pose.pose.position.z
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            path.poses.append(pose)

        return path

    def _prepare_guarded_path(self, msg: Path) -> Path:
        if not self.rotation_guard_enabled or not msg.poses:
            return self._prepare_path(msg)

        with self._lock:
            odom = self._latest_odom
            guard_active = self._rotation_guard_active

        if odom is None:
            return self._prepare_path(msg)

        is_stationary_xy = self._is_stationary_xy_path(msg)

        if self._is_pure_rotation_path(msg):
            self._reset_static_lock_state()
            target_yaw = self._path_last_yaw(msg)
            with self._lock:
                self._rotation_guard_active = True
                self._rotation_target_yaw = target_yaw
                self._rotation_release_ready_count = 0
                self._rotation_hold_start_time_s = 0.0
                self._last_rotation_hold_goal_time_s = 0.0
            return self._prepare_path(msg, anchor_xy_to_odom=True)

        if not guard_active:
            return self._prepare_path(msg)

        now_s = self._now_s()
        target_yaw = self._path_last_yaw(msg) if is_stationary_xy else self._path_first_yaw(msg)
        with self._lock:
            self._rotation_target_yaw = target_yaw
            if self._rotation_hold_start_time_s <= 0.0:
                self._rotation_hold_start_time_s = now_s
            hold_start_time_s = self._rotation_hold_start_time_s

        release_reached = self._rotation_release_reached(odom, target_yaw)
        yaw_error = abs(self._normalize_angle(target_yaw - self._odom_yaw(odom)))
        angular_speed = abs(float(odom.twist.twist.angular.z))
        hold_duration = max(0.0, now_s - hold_start_time_s)
        timed_release = (
            self.rotation_max_hold_duration > 0.0
            and hold_duration >= self.rotation_max_hold_duration
            and yaw_error <= self.rotation_timeout_release_yaw_error
            and angular_speed <= self.rotation_release_angular_speed
        )
        with self._lock:
            if release_reached:
                self._rotation_release_ready_count += 1
            else:
                self._rotation_release_ready_count = 0
            ready_count = self._rotation_release_ready_count

        if ready_count >= self.rotation_release_stable_samples or timed_release:
            with self._lock:
                self._rotation_guard_active = False
                self._rotation_release_ready_count = 0
                self._rotation_hold_start_time_s = 0.0
                self._last_rotation_hold_goal_time_s = 0.0
                self._force_next_goal_update = False
            if timed_release and ready_count < self.rotation_release_stable_samples:
                self.get_logger().warning(
                    "Releasing VLA translation after rotation hold timeout: "
                    f"yaw_error={math.degrees(yaw_error):.1f}deg "
                    f"wz={angular_speed:.3f} hold={hold_duration:.2f}s",
                    throttle_duration_sec=1.0,
                )
            else:
                self.get_logger().info("VLA rotation settled, releasing translation")
            return self._prepare_path(msg)

        with self._lock:
            if (
                self.rotation_hold_resend_period <= 0.0
                or now_s - self._last_rotation_hold_goal_time_s >= self.rotation_hold_resend_period
            ):
                self._force_next_goal_update = True
                self._last_rotation_hold_goal_time_s = now_s

        if ready_count < self.rotation_release_stable_samples:
            self.get_logger().info(
                "Holding VLA translation until rotation settles: "
                f"yaw_error={math.degrees(yaw_error):.1f}deg "
                f"wz={angular_speed:.3f} hold={hold_duration:.2f}s stable={ready_count}/"
                f"{self.rotation_release_stable_samples}",
                throttle_duration_sec=1.0,
            )
            return self._make_rotation_hold_path(msg, odom, target_yaw)

    def _prepare_path(self, msg: Path, *, anchor_xy_to_odom: bool = False) -> Path:
        if not msg.poses:
            return msg

        path = Path()
        path.header = msg.header
        path.poses = deepcopy(msg.poses)

        with self._lock:
            odom = self._latest_odom

        if anchor_xy_to_odom and odom is not None:
            for pose in path.poses:
                pose.pose.position.x = odom.pose.pose.position.x
                pose.pose.position.y = odom.pose.pose.position.y
                pose.pose.position.z = odom.pose.pose.position.z

        if not self.prepend_current_pose:
            return path

        if odom is None:
            return path

        first_frame = path.header.frame_id or path.poses[0].header.frame_id
        odom_frame = odom.header.frame_id
        if first_frame and odom_frame and first_frame != odom_frame:
            log = self.get_logger().error if self.strict_frame_check else self.get_logger().warning
            log(
                f"Cannot prepend odom pose because frame differs: path={first_frame} odom={odom_frame}"
            )
            return path

        current = type(path.poses[0])()
        current.header.stamp = path.header.stamp
        current.header.frame_id = first_frame or odom_frame
        current.pose = odom.pose.pose
        path.poses.insert(0, current)
        path.header.frame_id = current.header.frame_id
        return path

    def _should_send_goal_locked(self, path: Path, now_s: float) -> bool:
        """Must be called with _lock held."""
        if now_s - self._last_goal_time_s < self.min_goal_update_period:
            return False

        if self._force_next_goal_update:
            return True

        if self._last_goal_path is None or not self._last_goal_path.poses:
            return True

        prev = self._last_goal_path.poses[-1]
        curr = path.poses[-1]
        dx = curr.pose.position.x - prev.pose.position.x
        dy = curr.pose.position.y - prev.pose.position.y
        dyaw = abs(self._normalize_angle(self._yaw_from_pose_stamped(curr) - self._yaw_from_pose_stamped(prev)))
        return math.hypot(dx, dy) >= self.min_goal_delta_xy or dyaw >= self.min_goal_delta_yaw

    def _goal_response_cb(
        self,
        future,
        path: Path,
        sent_time_s: float,
        goal_generation: int,
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            with self._lock:
                is_current_goal = goal_generation == self._goal_generation
                if is_current_goal:
                    self._goal_in_flight = False
            if is_current_goal:
                self.get_logger().error(f"Failed to send FollowPath goal: {exc}")
            else:
                self.get_logger().debug(f"Ignoring stale FollowPath send failure: {exc}")
            return

        stale_goal = False
        with self._lock:
            if goal_generation != self._goal_generation:
                stale_goal = True
            else:
                self._goal_in_flight = False

        if stale_goal:
            if goal_handle.accepted:
                try:
                    cancel_future = goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(
                        lambda f: self._cancel_done_cb(f, "stale FollowPath goal"))
                except Exception as exc:
                    self.get_logger().warning(f"Failed to cancel stale FollowPath goal: {exc}")
            return

        if not goal_handle.accepted:
            self.get_logger().warning("FollowPath goal was rejected")
            return

        stale_after_accept = False
        with self._lock:
            if goal_generation != self._goal_generation:
                stale_after_accept = True
            else:
                self._goal_handle = goal_handle
                self._last_goal_path = path
                self._last_goal_time_s = sent_time_s

        if stale_after_accept:
            try:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda f: self._cancel_done_cb(f, "stale FollowPath goal"))
            except Exception as exc:
                self.get_logger().warning(f"Failed to cancel stale FollowPath goal: {exc}")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future: self._goal_result_cb(future, goal_handle))
        self.get_logger().debug(f"FollowPath goal accepted with {len(path.poses)} poses")

    def _goal_result_cb(self, future, goal_handle) -> None:
        try:
            future.result()
        except Exception as exc:
            self.get_logger().warning(f"FollowPath result failed: {exc}")
        with self._lock:
            if self._goal_handle is goal_handle:
                self._goal_handle = None

    def _timeout_check(self) -> None:
        try:
            with self._lock:
                goal_handle = self._goal_handle
                last_path_rx_time_s = self._last_path_rx_time_s

            if goal_handle is None or last_path_rx_time_s <= 0.0:
                return

            age_s = self._now_s() - last_path_rx_time_s
            should_stop = age_s > self.path_timeout and not self._stopped_for_current_gap

            if should_stop:
                self._stopped_for_current_gap = True

            if should_stop:
                self._stop_tracking(f"VLA path timeout ({age_s:.3f}s)", force=True)
        except Exception as exc:
            self.get_logger().error(f"Exception in _timeout_check: {exc}", throttle_duration_sec=2.0)

    def _publish_zero_velocity(self) -> None:
        zero = Twist()
        for _ in range(self.stop_publish_count):
            self._cmd_vel_pub.publish(zero)

    def _stop_tracking(self, reason: str, *, force: bool = False) -> None:
        with self._lock:
            self._last_goal_path = None
            self._last_goal_time_s = 0.0
            self._goal_generation += 1
            self._goal_in_flight = False
            self._rotation_guard_active = False
            self._rotation_release_ready_count = 0
            self._rotation_hold_start_time_s = 0.0
            self._last_rotation_hold_goal_time_s = 0.0
            self._force_next_goal_update = False
            self._static_lock_active = False
            self._static_lock_ready_count = 0
            self._static_lock_goal_valid = False

        if force:
            self._publish_zero_velocity()

        self._cancel_active_goal(reason)
        self.get_logger().info(f"Stopping VLA path tracking: {reason}")

    def _cancel_active_goal(self, reason: str) -> None:
        with self._lock:
            goal_handle = self._goal_handle
            if goal_handle is None:
                return
            self._goal_handle = None

        try:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda f: self._cancel_done_cb(f, reason)
            )
        except Exception as exc:
            self.get_logger().warning(f"Failed to cancel FollowPath goal for {reason}: {exc}")

    def _cancel_done_cb(self, future, reason: str) -> None:
        try:
            cancel_response = future.result()
            if cancel_response.goals_canceling:
                self.get_logger().info(f"Canceled FollowPath goal: {reason}")
            else:
                self.get_logger().warning(f"Failed to cancel FollowPath goal for {reason}")
        except Exception as exc:
            self.get_logger().warning(f"Cancel callback failed for {reason}: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VlaPathToNav2FollowPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
