#!/usr/bin/env python3
import copy
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath, SmoothPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class QueuedGoal:
    pose: PoseStamped
    use_supplied_orientation: bool


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def clone_pose_stamped(msg: PoseStamped) -> PoseStamped:
    return copy.deepcopy(msg)


class QueuedPathNavigationNode(Node):
    def __init__(self) -> None:
        super().__init__('queued_path_navigation')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('planner_action', 'compute_path_to_pose')
        self.declare_parameter('follow_path_action', 'follow_path')
        self.declare_parameter('smoother_action', 'smooth_path')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'general_goal_checker')
        self.declare_parameter('smoother_id', 'constrained_smoother')
        self.declare_parameter('enable_path_smoothing', True)
        self.declare_parameter('check_smoothing_collisions', True)
        self.declare_parameter('max_smoothing_duration_sec', 0.8)
        self.declare_parameter('use_goal_pose_topic', False)
        self.declare_parameter('timer_period_sec', 0.2)

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.plan_topic = self.get_parameter('plan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.controller_id = self.get_parameter('controller_id').value
        self.goal_checker_id = self.get_parameter('goal_checker_id').value
        self.smoother_id = self.get_parameter('smoother_id').value
        self.enable_path_smoothing = bool(self.get_parameter('enable_path_smoothing').value)
        self.check_smoothing_collisions = bool(self.get_parameter('check_smoothing_collisions').value)
        self.max_smoothing_duration_sec = float(self.get_parameter('max_smoothing_duration_sec').value)
        self.use_goal_pose_topic = bool(self.get_parameter('use_goal_pose_topic').value)
        timer_period = float(self.get_parameter('timer_period_sec').value)

        planner_action = self.get_parameter('planner_action').value
        follow_path_action = self.get_parameter('follow_path_action').value
        smoother_action = self.get_parameter('smoother_action').value

        self.plan_pub = self.create_publisher(Path, self.plan_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.point_sub = self.create_subscription(PointStamped, '/clicked_point', self.on_clicked_point, 10)
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, 10)
        self.goal_sub = None
        if self.use_goal_pose_topic:
            self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_pose, 10)

        self.enable_srv = self.create_service(SetBool, '~/enable', self.on_enable_request)
        self.clear_srv = self.create_service(Trigger, '~/clear', self.on_clear_request)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.compute_path_client = ActionClient(self, ComputePathToPose, planner_action)
        self.follow_path_client = ActionClient(self, FollowPath, follow_path_action)
        self.smooth_path_client = ActionClient(self, SmoothPath, smoother_action)
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.anchor_pose: Optional[PoseStamped] = None
        self.pending_start: Optional[PoseStamped] = None
        self.pending_goal: Optional[PoseStamped] = None
        self.goal_queue: Deque[QueuedGoal] = deque()
        self.raw_cumulative_path = Path()
        self.raw_cumulative_path.header.frame_id = self.map_frame
        self.cumulative_path = Path()
        self.cumulative_path.header.frame_id = self.map_frame

        self.path_version = 0
        self.is_planning = False
        self.is_smoothing = False
        self.is_executing = False
        self.auto_start_requested = False
        self.cancel_requested = False
        self.follow_goal_handle = None
        self.warning_times = {}

        self.get_logger().info(
            'Queued path navigation ready. Use RViz Publish Point to build a stitched path, '
            'call ~/enable to start motion, and call ~/clear to reset the queued path.')

    def on_timer(self) -> None:
        self.process_pending_operations()

    def now_msg(self):
        return self.get_clock().now().to_msg()

    @staticmethod
    def duration_from_seconds(seconds: float) -> Duration:
        sec = int(seconds)
        nanosec = int(max(0.0, seconds - sec) * 1e9)
        return Duration(sec=sec, nanosec=nanosec)

    def warn_throttled(self, key: str, message: str, period_sec: float = 5.0) -> None:
        now = time.monotonic()
        last = self.warning_times.get(key, 0.0)
        if now - last >= period_sec:
            self.warning_times[key] = now
            self.get_logger().warn(message)

    def current_robot_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except TransformException as ex:
            self.warn_throttled(
                'robot_pose',
                f'Waiting for transform {self.map_frame}->{self.robot_frame}: {ex}',
                period_sec=2.0,
            )
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = transform.header.stamp
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def publish_zero_twist(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def publish_path(self, path: Optional[Path] = None) -> None:
        if path is not None:
            self.cumulative_path = copy.deepcopy(path)
        self.plan_pub.publish(self.cumulative_path)

    def clear_queued_path(self, anchor_pose: Optional[PoseStamped] = None) -> None:
        self.path_version += 1
        self.goal_queue.clear()
        self.pending_start = None
        self.pending_goal = None
        self.is_planning = False
        self.is_smoothing = False
        self.raw_cumulative_path = Path()
        self.raw_cumulative_path.header.frame_id = self.map_frame
        self.raw_cumulative_path.header.stamp = self.now_msg()
        self.cumulative_path = Path()
        self.cumulative_path.header.frame_id = self.map_frame
        self.cumulative_path.header.stamp = self.raw_cumulative_path.header.stamp
        self.publish_path()
        self.anchor_pose = clone_pose_stamped(anchor_pose) if anchor_pose is not None else None

    def on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if self.is_executing:
            self.get_logger().warn('Ignoring /initialpose while path execution is active. Disable navigation first.')
            return

        anchor_pose = PoseStamped()
        anchor_pose.header = copy.deepcopy(msg.header)
        anchor_pose.pose = copy.deepcopy(msg.pose.pose)
        self.auto_start_requested = False
        self.clear_queued_path(anchor_pose=anchor_pose)
        self.get_logger().info('Reset queued path anchor from /initialpose')

    def on_clicked_point(self, msg: PointStamped) -> None:
        if self.is_executing:
            self.get_logger().warn('Ignoring clicked point while the robot is following a path')
            return

        goal = PoseStamped()
        goal.header.frame_id = msg.header.frame_id or self.map_frame
        goal.header.stamp = self.now_msg()
        goal.pose.position.x = msg.point.x
        goal.pose.position.y = msg.point.y
        goal.pose.position.z = msg.point.z
        goal.pose.orientation.w = 1.0

        self.goal_queue.append(QueuedGoal(goal, False))
        self.get_logger().info(
            f'Queued clicked point at ({msg.point.x:.2f}, {msg.point.y:.2f}); '
            f'{len(self.goal_queue)} segment(s) waiting to be planned')
        self.process_pending_operations()

    def on_goal_pose(self, msg: PoseStamped) -> None:
        if self.is_executing:
            self.get_logger().warn('Ignoring /goal_pose while the robot is following a path')
            return

        goal = clone_pose_stamped(msg)
        if not goal.header.frame_id:
            goal.header.frame_id = self.map_frame
        goal.header.stamp = self.now_msg()

        self.goal_queue.append(QueuedGoal(goal, True))
        self.get_logger().info(
            f'Queued goal pose; {len(self.goal_queue)} segment(s) waiting to be planned')
        self.process_pending_operations()

    def on_enable_request(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            if self.is_executing:
                response.success = True
                response.message = 'Path execution is already active.'
                return response

            if not self.cumulative_path.poses and not self.goal_queue and not self.is_planning and not self.is_smoothing:
                response.success = False
                response.message = 'No queued path. Use RViz Publish Point first.'
                return response

            self.auto_start_requested = True
            self.process_pending_operations()

            if self.is_planning or self.is_smoothing or self.goal_queue:
                response.success = True
                response.message = 'Navigation enabled. Waiting for queued path planning and smoothing to finish.'
            elif self.is_executing:
                response.success = True
                response.message = 'Navigation enabled. The robot started following the queued path.'
            else:
                response.success = True
                response.message = 'Navigation enabled. Waiting for the controller action server.'
            return response

        self.auto_start_requested = False
        if self.is_executing:
            self.request_follow_cancel()
            response.success = True
            response.message = 'Navigation disabled. Cancelling the active path and clearing it when the robot stops.'
        else:
            response.success = True
            response.message = 'Navigation disabled. Queued path kept for inspection.'
        return response

    def on_clear_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.auto_start_requested = False
        if self.is_executing:
            self.request_follow_cancel()
            response.success = True
            response.message = 'Cancelling the active path. The queued path will be cleared when execution stops.'
            return response

        self.clear_queued_path(anchor_pose=self.current_robot_pose())
        response.success = True
        response.message = 'Queued path cleared.'
        self.get_logger().info('Cleared queued path')
        return response

    def process_pending_operations(self) -> None:
        if self.cancel_requested or self.is_executing or self.is_smoothing:
            return

        if not self.is_planning and self.goal_queue:
            self.start_next_plan_request()
            return

        if (
            self.auto_start_requested
            and not self.is_planning
            and not self.is_smoothing
            and not self.goal_queue
            and self.cumulative_path.poses
        ):
            self.start_follow_path()

    def prepare_goal_pose(self, start: PoseStamped, queued_goal: QueuedGoal) -> PoseStamped:
        goal = clone_pose_stamped(queued_goal.pose)
        goal.header.stamp = self.now_msg()
        if not goal.header.frame_id:
            goal.header.frame_id = self.map_frame

        if queued_goal.use_supplied_orientation:
            return goal

        if goal.header.frame_id != start.header.frame_id:
            self.warn_throttled(
                'goal_frame_mismatch',
                f'Queued goal frame {goal.header.frame_id} differs from start frame {start.header.frame_id}; '
                'keeping the current orientation.',
            )
            goal.pose.orientation = copy.deepcopy(start.pose.orientation)
            return goal

        dx = goal.pose.position.x - start.pose.position.x
        dy = goal.pose.position.y - start.pose.position.y
        if abs(dx) > 1e-6 or abs(dy) > 1e-6:
            goal.pose.orientation = yaw_to_quaternion(math.atan2(dy, dx))
        else:
            goal.pose.orientation = copy.deepcopy(start.pose.orientation)
        return goal

    def start_next_plan_request(self) -> None:
        if not self.compute_path_client.wait_for_server(timeout_sec=0.1):
            self.warn_throttled('planner_server', 'Waiting for compute_path_to_pose action server...')
            return

        start = clone_pose_stamped(self.anchor_pose) if self.anchor_pose is not None else self.current_robot_pose()
        if start is None:
            return

        queued_goal = self.goal_queue.popleft()
        goal = self.prepare_goal_pose(start, queued_goal)

        action_goal = ComputePathToPose.Goal()
        action_goal.start = start
        action_goal.goal = goal
        action_goal.planner_id = ''
        action_goal.use_start = True

        self.pending_start = clone_pose_stamped(start)
        self.pending_goal = clone_pose_stamped(goal)
        self.is_planning = True
        request_version = self.path_version

        send_future = self.compute_path_client.send_goal_async(action_goal)
        send_future.add_done_callback(
            lambda future, version=request_version: self.on_plan_goal_response(future, version))

    def finish_plan_request(self) -> None:
        self.is_planning = False
        self.pending_start = None
        self.pending_goal = None

    def on_plan_goal_response(self, future, request_version: int) -> None:
        if request_version != self.path_version:
            return

        try:
            goal_handle = future.result()
        except Exception as ex:
            self.get_logger().warn(f'Failed to send planner goal: {ex}')
            self.finish_plan_request()
            self.process_pending_operations()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Planner rejected one queued segment')
            self.finish_plan_request()
            self.process_pending_operations()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result_future, version=request_version: self.on_plan_result(result_future, version))

    def on_plan_result(self, future, request_version: int) -> None:
        if request_version != self.path_version:
            return

        try:
            wrapped = future.result()
        except Exception as ex:
            self.get_logger().warn(f'Planner request failed: {ex}')
            self.finish_plan_request()
            self.process_pending_operations()
            return

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Planner request ended with status {self.status_to_text(wrapped.status)}')
            self.finish_plan_request()
            self.process_pending_operations()
            return

        path = wrapped.result.path
        if not path.poses:
            self.get_logger().warn('Planner returned an empty segment')
            self.finish_plan_request()
            self.process_pending_operations()
            return

        if self.raw_cumulative_path.poses:
            self.raw_cumulative_path.poses.extend(path.poses[1:])
            self.raw_cumulative_path.header = copy.deepcopy(path.header)
        else:
            self.raw_cumulative_path = copy.deepcopy(path)

        if self.pending_goal is not None:
            self.anchor_pose = clone_pose_stamped(self.pending_goal)
            self.anchor_pose.header = copy.deepcopy(path.header)

        self.get_logger().info(
            f'Queued raw path updated: {len(self.raw_cumulative_path.poses)} poses ready, '
            f'{len(self.goal_queue)} queued segment(s) still pending')

        self.finish_plan_request()
        self.request_smoothing()

    def request_smoothing(self) -> None:
        if not self.raw_cumulative_path.poses:
            self.publish_path(Path())
            self.process_pending_operations()
            return

        if not self.enable_path_smoothing or len(self.raw_cumulative_path.poses) < 3:
            self.publish_path(self.raw_cumulative_path)
            self.process_pending_operations()
            return

        if not self.smooth_path_client.wait_for_server(timeout_sec=0.1):
            self.warn_throttled(
                'smoother_server',
                'Waiting for smooth_path action server. Publishing the raw queued path for now.',
            )
            self.publish_path(self.raw_cumulative_path)
            self.process_pending_operations()
            return

        action_goal = SmoothPath.Goal()
        action_goal.path = copy.deepcopy(self.raw_cumulative_path)
        action_goal.smoother_id = self.smoother_id
        action_goal.max_smoothing_duration = self.duration_from_seconds(self.max_smoothing_duration_sec)
        action_goal.check_for_collisions = self.check_smoothing_collisions

        self.is_smoothing = True
        request_version = self.path_version
        send_future = self.smooth_path_client.send_goal_async(action_goal)
        send_future.add_done_callback(
            lambda future, version=request_version: self.on_smooth_goal_response(future, version))

    def finish_smoothing(self) -> None:
        self.is_smoothing = False

    def on_smooth_goal_response(self, future, request_version: int) -> None:
        if request_version != self.path_version:
            return

        try:
            goal_handle = future.result()
        except Exception as ex:
            self.get_logger().warn(f'Failed to send smooth_path goal: {ex}')
            self.finish_smoothing()
            self.publish_path(self.raw_cumulative_path)
            self.process_pending_operations()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('smooth_path goal was rejected. Using the raw queued path.')
            self.finish_smoothing()
            self.publish_path(self.raw_cumulative_path)
            self.process_pending_operations()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result_future, version=request_version: self.on_smooth_result(result_future, version))

    def on_smooth_result(self, future, request_version: int) -> None:
        if request_version != self.path_version:
            return

        try:
            wrapped = future.result()
        except Exception as ex:
            self.get_logger().warn(f'smooth_path request failed: {ex}. Using the raw queued path.')
            self.finish_smoothing()
            self.publish_path(self.raw_cumulative_path)
            self.process_pending_operations()
            return

        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.was_completed and wrapped.result.path.poses:
            self.publish_path(wrapped.result.path)
            self.get_logger().info(
                f'Published smoothed queued path with {len(self.cumulative_path.poses)} poses')
        else:
            self.get_logger().warn(
                f'smooth_path ended with status {self.status_to_text(wrapped.status)}. Using the raw queued path.')
            self.publish_path(self.raw_cumulative_path)

        self.finish_smoothing()
        self.process_pending_operations()

    def start_follow_path(self) -> None:
        if not self.follow_path_client.wait_for_server(timeout_sec=0.1):
            self.warn_throttled('follow_path_server', 'Waiting for follow_path action server...')
            return

        action_goal = FollowPath.Goal()
        action_goal.path = copy.deepcopy(self.cumulative_path)
        action_goal.controller_id = self.controller_id
        action_goal.goal_checker_id = self.goal_checker_id

        self.is_executing = True
        self.cancel_requested = False
        send_future = self.follow_path_client.send_goal_async(action_goal)
        send_future.add_done_callback(self.on_follow_goal_response)

    def on_follow_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as ex:
            self.get_logger().warn(f'Failed to send follow_path goal: {ex}')
            self.follow_goal_handle = None
            self.is_executing = False
            self.cancel_requested = False
            self.auto_start_requested = False
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('follow_path goal was rejected by the controller server')
            self.follow_goal_handle = None
            self.is_executing = False
            self.cancel_requested = False
            self.auto_start_requested = False
            return

        self.follow_goal_handle = goal_handle
        self.get_logger().info(f'Started following queued path with {len(self.cumulative_path.poses)} poses')

        if self.cancel_requested:
            cancel_future = self.follow_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.on_follow_cancel_response)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_follow_result)

    def request_follow_cancel(self) -> None:
        if not self.is_executing:
            return
        if self.cancel_requested:
            return

        self.cancel_requested = True
        self.auto_start_requested = False
        self.publish_zero_twist()

        if self.follow_goal_handle is None:
            self.get_logger().info('Requested cancellation. Waiting for follow_path goal acceptance first.')
            return

        self.get_logger().info('Cancelling active queued-path execution')
        cancel_future = self.follow_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.on_follow_cancel_response)

    def on_follow_cancel_response(self, future) -> None:
        try:
            future.result()
        except Exception as ex:
            self.get_logger().warn(f'follow_path cancellation request failed: {ex}')

    def on_follow_result(self, future) -> None:
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as ex:
            self.get_logger().warn(f'follow_path result handling failed: {ex}')
            status = GoalStatus.STATUS_ABORTED

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Queued path execution finished successfully')
        else:
            self.get_logger().warn(
                f'Queued path execution ended with status {self.status_to_text(status)}')

        self.publish_zero_twist()
        self.follow_goal_handle = None
        self.is_executing = False
        self.cancel_requested = False
        self.auto_start_requested = False
        self.clear_queued_path(anchor_pose=self.current_robot_pose())

    @staticmethod
    def status_to_text(status: int) -> str:
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, f'CODE_{status}')


def main() -> None:
    rclpy.init()
    node = QueuedPathNavigationNode()
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
