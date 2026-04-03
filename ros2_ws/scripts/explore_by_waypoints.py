#!/usr/bin/env python3
"""
Exploração por waypoints via fleet (go_to_point + gravação de rota).

Uso (workspace e ROS 2 já sourceados):
  cd /path/to/ros2_ws/ros2_ws
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  # Em simulação (default): use_sim_time=true. Robô real: acrescente --wall-time.
  python3 scripts/explore_by_waypoints.py \\
    --single-robot \\
    --route exploracao1 \\
    --initial-pose "0.0,0.0,0.0" \\
    --points "0.5,0,0;1.0,0,0;1.0,0.5,0;0.5,0.5,0" \\
    --require-motion \\
    --motion-min-dist 0.10 \\
    --motion-timeout 10 \\
    --export exp_exploracao1.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from fleet_msgs.msg import FleetStatus
from fleet_msgs.srv import (
    DisableCollection,
    EnableCollection,
    GoToPoint,
    StartRecord,
    StopRecord,
)


@dataclass
class WaypointResult:
    index: int
    x: float
    y: float
    yaw: float
    go_to_point_ok: bool
    go_to_point_message: str
    nav_started: bool
    nav_idle: bool
    motion_ok: bool
    motion_m: float


def _parse_xyyaw(s: str) -> Tuple[float, float, float]:
    parts = [p.strip() for p in s.split(",")]
    if len(parts) != 3:
        raise ValueError(f"esperado x,y,yaw, veio: {s!r}")
    return float(parts[0]), float(parts[1]), float(parts[2])


def _parse_points(s: str) -> List[Tuple[float, float, float]]:
    out: List[Tuple[float, float, float]] = []
    for chunk in s.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        out.append(_parse_xyyaw(chunk))
    if not out:
        raise ValueError("--points vazio ou inválido")
    return out


def _robot_matches(rs_robot_id: str, target: str) -> bool:
    if target:
        return rs_robot_id == target
    return rs_robot_id in ("", "default")


class ExploreByWaypoints(Node):
    def __init__(self) -> None:
        super().__init__("explore_by_waypoints")
        self._status: Optional[FleetStatus] = None
        self._odom: Optional[Odometry] = None
        self.create_subscription(FleetStatus, "fleet/status", self._status_cb, 10)
        self._odom_sub = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _status_cb(self, msg: FleetStatus) -> None:
        self._status = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    def enable_odom(self, topic: str) -> None:
        self._odom_sub = self.create_subscription(Odometry, topic, self._odom_cb, 10)

    def call_srv(self, srv_type, name: str, request, timeout_sec: float = 8.0):
        cli = self.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=timeout_sec):
            return None, False, f"service {name} indisponível"
        fut = cli.call_async(request)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        if not fut.done():
            return None, False, f"timeout {name}"
        try:
            return fut.result(), True, ""
        except Exception as e:
            return None, False, str(e)

    def wait_nav_state(self, robot_id: str, expected: str, timeout_sec: float) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._status is None:
                continue
            for rs in self._status.robots:
                if _robot_matches(rs.robot_id, robot_id) and rs.nav_state == expected:
                    return True
            time.sleep(0.05)
        return False

    def odom_xy(self) -> Optional[Tuple[float, float]]:
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        return (float(p.x), float(p.y))

    def wait_odom_sample(self, timeout_sec: float) -> Optional[Tuple[float, float]]:
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            xy = self.odom_xy()
            if xy is not None:
                return xy
        return None

    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        time.sleep(0.3)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy
        for i in range(36):
            msg.pose.covariance[i] = 0.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942
        pub.publish(msg)
        self.get_logger().info(f"Publicado /initialpose map=({x}, {y}, yaw={yaw})")

    def wait_tf_map_base(self, map_frame: str, base_frame: str, timeout_sec: float) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._tf_buffer.can_transform(map_frame, base_frame, rclpy.time.Time()):
                return True
            try:
                from rclpy.duration import Duration

                self._tf_buffer.lookup_transform(
                    map_frame, base_frame, rclpy.time.Time(), timeout=Duration(seconds=0, nanoseconds=1)
                )
                return True
            except Exception:
                pass
        return False


def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def main() -> int:
    p = argparse.ArgumentParser(description="Exploração por waypoints (fleet + Nav2)")
    p.add_argument("--single-robot", action="store_true", help="robot_id vazio (sim típico)")
    p.add_argument("--robot", default="", help="robot_id explícito (ignorado se --single-robot)")
    p.add_argument("--route", required=True, help="nome da rota para start_record")
    p.add_argument("--points", required=True, help="waypoints: x,y,yaw separados por ;")
    p.add_argument("--initial-pose", default="", help='opcional: "x,y,yaw" em map para /initialpose')
    p.add_argument("--skip-collection", action="store_true", help="não chama enable/disable collection")
    p.add_argument(
        "--topics",
        default="scan,odom,imu",
        help="tópicos para enable_collection (vírgula)",
    )
    p.add_argument("--wait-collection", type=float, default=2.0)
    p.add_argument("--wait-goal", type=float, default=120.0, help="timeout navigating->idle por waypoint")
    p.add_argument("--nav-start-timeout", type=float, default=15.0, help="timeout até nav_state=navigating")
    p.add_argument("--require-motion", action="store_true", help="exige deslocamento em odometria")
    p.add_argument("--motion-min-dist", type=float, default=0.08, help="metros (plano XY)")
    p.add_argument("--motion-timeout", type=float, default=15.0, help="tempo para amostrar odom após idle")
    p.add_argument("--odom-topic", default="/odom", help="tópico de odometria para --require-motion")
    p.add_argument("--map-frame", default="map", help="frame para TF wait")
    p.add_argument("--base-frame", default="base_link", help="frame base para TF wait")
    p.add_argument("--wait-tf", type=float, default=30.0, help="espera map->base (0 desliga)")
    p.add_argument("--export", default="", help="ficheiro JSON com resumo")
    p.add_argument(
        "--wall-time",
        action="store_true",
        help="relógio real (robô físico). Por omissão usa use_sim_time=true (Gazebo/Nav2 com /clock).",
    )
    args = p.parse_args()

    robot_id = "" if args.single_robot else (args.robot or "")
    points = _parse_points(args.points)

    # Sem isto, TF do Gazebo (tempos de sim) vs relógio de parede → TF_OLD_DATA em base_footprint etc.
    init_argv = [sys.argv[0]]
    if not args.wall_time:
        init_argv += ["--ros-args", "-p", "use_sim_time:=true"]
    rclpy.init(args=init_argv)
    node = ExploreByWaypoints()
    results: List[WaypointResult] = []
    summary = {
        "robot_id": robot_id,
        "route": args.route,
        "points": [list(pt) for pt in points],
        "ok": True,
        "errors": [],
    }

    try:
        if args.require_motion:
            node.enable_odom(args.odom_topic)

        if args.wait_tf > 0:
            if not node.wait_tf_map_base(args.map_frame, args.base_frame, args.wait_tf):
                summary["errors"].append(
                    f"TF {args.map_frame}->{args.base_frame} não disponível em {args.wait_tf}s"
                )
                summary["ok"] = False

        if args.initial_pose.strip():
            ix, iy, iyaw = _parse_xyyaw(args.initial_pose.strip())
            node.publish_initial_pose(ix, iy, iyaw)
            time.sleep(1.0)

        if not args.skip_collection:
            req_e = EnableCollection.Request()
            req_e.robot_id = robot_id
            req_e.topics = [t.strip() for t in args.topics.split(",") if t.strip()]
            req_e.output_mode = "rosbag2"
            resp, ok, err = node.call_srv(EnableCollection, "enable_collection", req_e)
            if not (ok and resp and resp.success):
                summary["errors"].append(f"enable_collection: {err or (resp.message if resp else '')}")
                summary["ok"] = False
            time.sleep(args.wait_collection)

        req_sr = StartRecord.Request()
        req_sr.robot_id = robot_id
        req_sr.route_name = args.route
        resp, ok, err = node.call_srv(StartRecord, "start_record", req_sr)
        if not (ok and resp and resp.success):
            summary["errors"].append(f"start_record: {err or (resp.message if resp else '')}")
            summary["ok"] = False
            if args.export:
                with open(args.export, "w", encoding="utf-8") as f:
                    json.dump({**summary, "waypoints": [asdict(r) for r in results]}, f, indent=2)
            return 1

        for i, (wx, wy, wyaw) in enumerate(points, start=1):
            pos_before: Optional[Tuple[float, float]] = None
            if args.require_motion:
                pos_before = node.wait_odom_sample(3.0)

            req_g = GoToPoint.Request()
            req_g.robot_id = robot_id
            req_g.x = wx
            req_g.y = wy
            req_g.yaw = wyaw
            resp_g, ok_g, err_g = node.call_srv(GoToPoint, "go_to_point", req_g, timeout_sec=15.0)
            g_ok = ok_g and resp_g is not None and resp_g.success
            g_msg = (resp_g.message if resp_g else err_g) or ""

            nav_started = False
            nav_idle = False
            motion_ok = True
            motion_m = 0.0

            if g_ok:
                nav_started = node.wait_nav_state(robot_id, "navigating", args.nav_start_timeout)
                nav_idle = node.wait_nav_state(robot_id, "idle", args.wait_goal)

            if args.require_motion and g_ok:
                pos_after = node.wait_odom_sample(args.motion_timeout)
                if pos_before is None or pos_after is None:
                    motion_ok = False
                    motion_m = -1.0
                else:
                    motion_m = _dist(pos_before, pos_after)
                    motion_ok = motion_m >= args.motion_min_dist

            wr = WaypointResult(
                index=i,
                x=wx,
                y=wy,
                yaw=wyaw,
                go_to_point_ok=g_ok,
                go_to_point_message=g_msg,
                nav_started=nav_started,
                nav_idle=nav_idle,
                motion_ok=motion_ok,
                motion_m=motion_m,
            )
            results.append(wr)

            if not g_ok or not nav_started or not nav_idle:
                summary["ok"] = False
                summary["errors"].append(f"wp{i}: go={g_ok} nav_start={nav_started} idle={nav_idle} msg={g_msg}")
            if args.require_motion and not motion_ok:
                summary["ok"] = False
                summary["errors"].append(f"wp{i}: motion {motion_m:.3f}m < {args.motion_min_dist}")

        req_st = StopRecord.Request()
        req_st.robot_id = robot_id
        resp, ok, err = node.call_srv(StopRecord, "stop_record", req_st)
        stop_ok = ok and resp is not None and resp.success
        if not stop_ok:
            summary["ok"] = False
            summary["errors"].append(f"stop_record: {err or (resp.message if resp else '')}")
        elif resp and ("Route length: 0" in resp.message or "Route length:0" in resp.message):
            summary["ok"] = False
            summary["errors"].append(f"stop_record: rota vazia — {resp.message}")

        if not args.skip_collection:
            req_d = DisableCollection.Request()
            req_d.robot_id = robot_id
            resp_d, ok_d, err_d = node.call_srv(DisableCollection, "disable_collection", req_d)
            if not (ok_d and resp_d and resp_d.success):
                summary["ok"] = False
                summary["errors"].append(f"disable_collection: {err_d or (resp_d.message if resp_d else '')}")

        out = {**summary, "waypoints": [asdict(r) for r in results]}
        if args.export:
            with open(args.export, "w", encoding="utf-8") as f:
                json.dump(out, f, indent=2)
            print(f"Exportado: {args.export}")

        for r in results:
            print(
                f"wp{r.index}: go_to_point={r.go_to_point_ok} nav {r.nav_started}->{r.nav_idle} "
                f"motion={r.motion_m:.3f}m ok={r.motion_ok}"
            )
        return 0 if summary["ok"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
