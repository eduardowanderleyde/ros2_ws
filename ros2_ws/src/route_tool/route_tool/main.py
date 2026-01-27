#!/usr/bin/env python3
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener, TransformException

from nav2_msgs.action import NavigateThroughPoses

import yaml


@dataclass
class XYYaw:
    x: float
    y: float
    yaw: float


def quat_to_yaw(q) -> float:
    """Convert geometry_msgs/Quaternion to yaw (rad)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class RouteTool(Node):
    def __init__(self) -> None:
        super().__init__('route_tool')

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('record_rate_hz', 5.0)

        # Filters
        self.declare_parameter('min_dist_m', 0.10)  # save if moved >= 10 cm
        self.declare_parameter('min_yaw_deg', 5.0)  # or rotated >= 5 degrees

        # Saving
        self.declare_parameter('routes_dir', 'routes')
        self.declare_parameter('route_name', 'route1')

        # Nav2 action name (default used by Nav2)
        self.declare_parameter('nav2_action_name', '/navigate_through_poses')

        self.global_frame: str = self.get_parameter('global_frame').value
        self.base_frame: str = self.get_parameter('base_frame').value
        self.record_rate_hz: float = float(self.get_parameter('record_rate_hz').value)

        self.min_dist_m: float = float(self.get_parameter('min_dist_m').value)
        self.min_yaw_rad: float = math.radians(float(self.get_parameter('min_yaw_deg').value))

        self.routes_dir: str = self.get_parameter('routes_dir').value
        self.route_name: str = self.get_parameter('route_name').value

        self.nav2_action_name: str = self.get_parameter('nav2_action_name').value

        # ----------------------------
        # TF
        # ----------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ----------------------------
        # Route state
        # ----------------------------
        self.route: List[PoseStamped] = []
        self.is_recording: bool = False
        self.is_navigating: bool = False
        self._last_saved: Optional[XYYaw] = None

        # ----------------------------
        # Timer (record)
        # ----------------------------
        record_period = 1.0 / max(self.record_rate_hz, 0.1)
        self.record_timer = self.create_timer(record_period, self._record_timer_callback)

        # ----------------------------
        # Services
        # ----------------------------
        self.start_srv = self.create_service(Trigger, 'start_route', self._handle_start_route)
        self.stop_srv = self.create_service(Trigger, 'stop_route', self._handle_stop_route)
        self.play_srv = self.create_service(Trigger, 'play_route', self._handle_play_route)

        # ----------------------------
        # Nav2 Action Client
        # ----------------------------
        self.nav2_client = ActionClient(self, NavigateThroughPoses, self.nav2_action_name)
        self._goal_handle = None
        self._last_feedback_log_time = self.get_clock().now()

        self.get_logger().info('route_tool ready (record + save + play via Nav2 Action).')

    # -------------------------------------------------------------------------
    # Services
    # -------------------------------------------------------------------------
    def _handle_start_route(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.is_navigating:
            response.success = False
            response.message = 'Cannot start recording while navigating.'
            return response

        self.route.clear()
        self._last_saved = None
        self.is_recording = True

        self.get_logger().info('Recording started.')
        response.success = True
        response.message = 'Recording started.'
        return response

    def _handle_stop_route(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.is_recording = False

        count = len(self.route)
        saved_path = None
        if count > 0:
            saved_path = self._save_route_yaml()

        msg = f'Recording stopped. Route length: {count}.'
        if saved_path:
            msg += f' Saved: {saved_path}'
        self.get_logger().info(msg)

        response.success = True
        response.message = msg
        return response

    def _handle_play_route(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.is_recording:
            response.success = False
            response.message = 'Stop recording before playing.'
            return response

        if self.is_navigating:
            response.success = False
            response.message = 'Already navigating.'
            return response

        if not self.route:
            self.get_logger().warn('Play requested but route is empty.')
            response.success = False
            response.message = 'Route is empty.'
            return response

        # Wait briefly for the Nav2 action server
        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f'Nav2 action server not available: {self.nav2_action_name}')
            response.success = False
            response.message = f'Nav2 action server not available: {self.nav2_action_name}'
            return response

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.route  # list[PoseStamped] in frame "map"
        # goal_msg.behavior_tree = ""  # optional: custom BT xml path

        self.is_navigating = True
        self.get_logger().info(f'Sending route to Nav2: {len(self.route)} poses.')

        send_future = self.nav2_client.send_goal_async(goal_msg, feedback_callback=self._nav2_feedback_cb)
        send_future.add_done_callback(self._nav2_goal_response_cb)

        response.success = True
        response.message = 'Nav2 navigation started.'
        return response

    # -------------------------------------------------------------------------
    # Record timer
    # -------------------------------------------------------------------------
    def _record_timer_callback(self) -> None:
        if not self.is_recording:
            return

        pose = self._get_current_pose()
        if pose is None:
            return

        if self._should_save_pose(pose):
            self.route.append(pose)
            self._last_saved = self._pose_to_xyyaw(pose)

    # -------------------------------------------------------------------------
    # Nav2 callbacks
    # -------------------------------------------------------------------------
    def _nav2_goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal.')
            self.is_navigating = False
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Nav2 goal accepted. Navigating...')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_cb)

    def _nav2_result_cb(self, future) -> None:
        try:
            result = future.result().result
            status = future.result().status
        except Exception as ex:
            self.get_logger().error(f'Failed to get Nav2 result: {ex}')
            self.is_navigating = False
            return

        # status codes are in action_msgs/GoalStatus
        self.get_logger().info(f'Nav2 finished. status={status} result={result}')
        self.is_navigating = False
        self._goal_handle = None

    def _nav2_feedback_cb(self, feedback_msg) -> None:
        # Throttle feedback logging (avoid spamming)
        now = self.get_clock().now()
        if (now - self._last_feedback_log_time).nanoseconds < int(2e9):
            return
        self._last_feedback_log_time = now

        fb = feedback_msg.feedback
        # NavigateThroughPoses feedback provides current_pose, distance_remaining, etc (varies by version)
        # We log minimal info safely.
        try:
            self.get_logger().info(f'Nav2 feedback: distance_remaining={fb.distance_remaining:.2f}')
        except Exception:
            self.get_logger().info('Nav2 feedback received.')

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _get_current_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                Time(),  # latest
                timeout=Duration(seconds=0.2),
            )
        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed {self.global_frame}->{self.base_frame}: {ex}')
            return None

        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = self.global_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _pose_to_xyyaw(self, pose: PoseStamped) -> XYYaw:
        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)
        yaw = quat_to_yaw(pose.pose.orientation)
        return XYYaw(x=x, y=y, yaw=yaw)

    def _should_save_pose(self, pose: PoseStamped) -> bool:
        curr = self._pose_to_xyyaw(pose)
        if self._last_saved is None:
            return True

        dx = curr.x - self._last_saved.x
        dy = curr.y - self._last_saved.y
        dist = math.hypot(dx, dy)

        dyaw = wrap_angle(curr.yaw - self._last_saved.yaw)
        return (dist >= self.min_dist_m) or (abs(dyaw) >= self.min_yaw_rad)

    def _save_route_yaml(self) -> Optional[str]:
        os.makedirs(self.routes_dir, exist_ok=True)
        path = os.path.join(self.routes_dir, f'{self.route_name}.yaml')

        data = {
            'route_name': self.route_name,
            'frame': self.global_frame,
            'poses': []
        }

        for p in self.route:
            yaw = quat_to_yaw(p.pose.orientation)
            data['poses'].append({
                'x': float(p.pose.position.x),
                'y': float(p.pose.position.y),
                'yaw': float(yaw),
            })

        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, sort_keys=False)

        return path


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteTool()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down (KeyboardInterrupt).')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
