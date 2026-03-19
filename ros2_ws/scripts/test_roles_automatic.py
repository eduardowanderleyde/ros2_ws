#!/usr/bin/env python3
"""
Teste automatizado por papéis (MUUT/FUUT/SU), sem UI.

Fluxo:
  1) Habilita coleta para MUUT, FUUT, SU.
  2) Tenta movimento em FUUT e SU (deve falhar com ROLE_NOT_ALLOWED).
  3) MUUT: start_record -> sequência de go_to_point -> stop_record.
  4) MUUT: play_route da rota recém gravada.
  5) Desabilita coleta em todos e imprime resumo.

Uso:
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  python3 scripts/test_roles_automatic.py
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from fleet_msgs.msg import FleetStatus
from fleet_msgs.srv import (
    CollectionStatus,
    DisableCollection,
    EnableCollection,
    GoToPoint,
    ListRoutes,
    PlayRoute,
    StartRecord,
    StopRecord,
)


class RoleAutoTester(Node):
    def __init__(self) -> None:
        super().__init__("test_roles_automatic")
        self.last_status: Optional[FleetStatus] = None
        self.create_subscription(FleetStatus, "fleet/status", self._status_cb, 10)

    def _status_cb(self, msg: FleetStatus) -> None:
        self.last_status = msg

    def call_srv(self, srv_type, name: str, request, timeout_sec: float = 8.0):
        cli = self.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=timeout_sec):
            return None, False, f"service {name} indisponível"
        fut = cli.call_async(request)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        if not fut.done():
            return None, False, f"timeout {name}"
        try:
            resp = fut.result()
            return resp, True, ""
        except Exception as e:
            return None, False, str(e)

    def wait_nav_state(self, robot_id: str, expected: str, timeout_sec: float = 30.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.last_status is None:
                continue
            for rs in self.last_status.robots:
                rid = rs.robot_id if rs.robot_id else "default"
                if rid == robot_id and rs.nav_state == expected:
                    return True
            time.sleep(0.1)
        return False


def _print_case(name: str, ok: bool, detail: str = "") -> None:
    status = "OK" if ok else "FALHOU"
    print(f"[{status}] {name}{(': ' + detail) if detail else ''}")


def _expect_role_denied(resp, role_name: str) -> Tuple[bool, str]:
    if resp is None:
        return False, "sem resposta"
    if getattr(resp, "success", True):
        return False, f"{role_name} aceitou operação de movimento indevidamente"
    code = getattr(resp, "error_code", "")
    if code != "ROLE_NOT_ALLOWED":
        return False, f"error_code esperado ROLE_NOT_ALLOWED, veio {code!r}"
    return True, f"{role_name} bloqueado corretamente ({code})"


def main() -> int:
    p = argparse.ArgumentParser(description="Teste automático MUUT/FUUT/SU sem UI")
    p.add_argument("--muut", default="tb1")
    p.add_argument("--fuut", default="tb2")
    p.add_argument("--su", default="tb3")
    p.add_argument("--route", default="role_demo_r1")
    p.add_argument("--wait-collection", type=float, default=3.0)
    p.add_argument("--wait-goal", type=float, default=45.0)
    args = p.parse_args()

    rclpy.init()
    node = RoleAutoTester()
    ok_count = 0
    fail_count = 0

    try:
        print("\n== Teste automático por roles ==")
        print(f"MUUT={args.muut} FUUT={args.fuut} SU={args.su}")

        # 1) Enable collection para todos
        for rid in [args.muut, args.fuut, args.su]:
            req = EnableCollection.Request()
            req.robot_id = rid
            req.topics = ["scan", "odom", "imu"]
            req.output_mode = "rosbag2"
            resp, ok, err = node.call_srv(EnableCollection, "enable_collection", req)
            this_ok = ok and resp is not None and resp.success
            _print_case(f"enable_collection({rid})", this_ok, (resp.message if resp else err))
            ok_count += int(this_ok)
            fail_count += int(not this_ok)

        time.sleep(args.wait_collection)

        # 2) FUUT/SU não podem mover
        for rid, role in [(args.fuut, "FUUT"), (args.su, "SU")]:
            req = GoToPoint.Request()
            req.robot_id = rid
            req.x = 0.5
            req.y = 0.0
            req.yaw = 0.0
            resp, ok, err = node.call_srv(GoToPoint, "go_to_point", req)
            this_ok, detail = _expect_role_denied(resp if ok else None, role)
            if not ok:
                detail = err
            _print_case(f"go_to_point bloqueado em {rid}/{role}", this_ok, detail)
            ok_count += int(this_ok)
            fail_count += int(not this_ok)

        # 3) MUUT grava rota com go_to_point automático
        req_sr = StartRecord.Request()
        req_sr.robot_id = args.muut
        req_sr.route_name = args.route
        resp, ok, err = node.call_srv(StartRecord, "start_record", req_sr)
        this_ok = ok and resp is not None and resp.success
        _print_case(f"start_record({args.muut})", this_ok, (resp.message if resp else err))
        ok_count += int(this_ok)
        fail_count += int(not this_ok)

        waypoints = [(0.6, 0.0), (0.6, 0.6), (0.0, 0.6), (0.0, 0.0)]
        if this_ok:
            for i, (x, y) in enumerate(waypoints, start=1):
                req_g = GoToPoint.Request()
                req_g.robot_id = args.muut
                req_g.x = x
                req_g.y = y
                req_g.yaw = 0.0
                resp_g, ok_g, err_g = node.call_srv(GoToPoint, "go_to_point", req_g, timeout_sec=12.0)
                sent_ok = ok_g and resp_g is not None and resp_g.success
                _print_case(f"go_to_point MUUT wp{i}", sent_ok, (resp_g.message if resp_g else err_g))
                ok_count += int(sent_ok)
                fail_count += int(not sent_ok)
                if not sent_ok:
                    continue

                nav_started = node.wait_nav_state(args.muut, "navigating", timeout_sec=8.0)
                nav_done = node.wait_nav_state(args.muut, "idle", timeout_sec=args.wait_goal)
                wp_ok = nav_started and nav_done
                _print_case(f"wp{i} execução", wp_ok, "navigating->idle")
                ok_count += int(wp_ok)
                fail_count += int(not wp_ok)

        req_st = StopRecord.Request()
        req_st.robot_id = args.muut
        resp, ok, err = node.call_srv(StopRecord, "stop_record", req_st)
        this_ok = ok and resp is not None and resp.success and "Route length: 0" not in resp.message
        _print_case(f"stop_record({args.muut})", this_ok, (resp.message if resp else err))
        ok_count += int(this_ok)
        fail_count += int(not this_ok)

        # valida rota salva
        req_lr = ListRoutes.Request()
        req_lr.robot_id = args.muut
        resp, ok, err = node.call_srv(ListRoutes, "list_routes", req_lr)
        has_route = ok and resp is not None and (args.route in list(resp.route_names))
        _print_case(f"list_routes contém {args.route}", has_route, str(list(resp.route_names)) if resp else err)
        ok_count += int(has_route)
        fail_count += int(not has_route)

        # 4) Play route no MUUT
        req_pr = PlayRoute.Request()
        req_pr.robot_id = args.muut
        req_pr.route_name = args.route
        resp, ok, err = node.call_srv(PlayRoute, "play_route", req_pr, timeout_sec=12.0)
        play_ok = ok and resp is not None and resp.success
        _print_case(f"play_route({args.muut})", play_ok, (resp.message if resp else err))
        ok_count += int(play_ok)
        fail_count += int(not play_ok)
        if play_ok:
            nav_done = node.wait_nav_state(args.muut, "idle", timeout_sec=max(args.wait_goal, 60.0))
            _print_case("play_route terminou", nav_done, "estado voltou para idle")
            ok_count += int(nav_done)
            fail_count += int(not nav_done)

        # 5) Disable collection para todos
        for rid in [args.muut, args.fuut, args.su]:
            req = DisableCollection.Request()
            req.robot_id = rid
            resp, ok, err = node.call_srv(DisableCollection, "disable_collection", req)
            this_ok = ok and resp is not None and resp.success
            _print_case(f"disable_collection({rid})", this_ok, (resp.message if resp else err))
            ok_count += int(this_ok)
            fail_count += int(not this_ok)

            req_cs = CollectionStatus.Request()
            req_cs.robot_id = rid
            resp_cs, ok_cs, err_cs = node.call_srv(CollectionStatus, "collection_status", req_cs)
            cs_ok = ok_cs and resp_cs is not None and not resp_cs.is_collecting
            details = (
                f"is_collecting={resp_cs.is_collecting} bytes={resp_cs.bytes_written}"
                if resp_cs is not None
                else err_cs
            )
            _print_case(f"collection_status OFF ({rid})", cs_ok, details)
            ok_count += int(cs_ok)
            fail_count += int(not cs_ok)

        print("\n" + "=" * 60)
        print(f"Resumo final: {ok_count} OK, {fail_count} FALHAS")
        print("=" * 60)
        return 0 if fail_count == 0 else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

