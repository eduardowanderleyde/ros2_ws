#!/usr/bin/env python3
"""
Script para testar casos do fleet (orchestrator + data collector).

Uso (com workspace e ROS 2 já sourceados):
  cd /path/to/ros2_ws
  source install/setup.bash
  python3 scripts/test_fleet_cases.py [--robot tb1] [--route r1] [--no-record] [--no-collection] [--with-play]

Cenários:
  1) list_robots
  2) list_routes(robot)
  3) start_record(robot, route) -> espera 2s -> stop_record(robot)
  4) enable_collection(robot) -> espera 5s -> disable_collection(robot)
  5) collection_status(robot)
  6) [opcional] play_route(robot, route) se --with-play
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from fleet_msgs.srv import (
    StartRecord,
    StopRecord,
    PlayRoute,
    ListRobots,
    ListRoutes,
    EnableCollection,
    DisableCollection,
    CollectionStatus,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Testa serviços do fleet (orchestrator + collector).")
    parser.add_argument("--robot", default="tb1", help="robot_id para testes")
    parser.add_argument("--route", default="r1", help="nome da rota para record/play")
    parser.add_argument("--no-record", action="store_true", help="pula testes de record (start/stop)")
    parser.add_argument("--no-collection", action="store_true", help="pula testes de coleta (enable/disable)")
    parser.add_argument("--with-play", action="store_true", help="inclui play_route (requer Nav2 e rota salva)")
    parser.add_argument("--wait-record", type=float, default=2.0, help="segundos entre start_record e stop_record")
    parser.add_argument("--wait-collection", type=float, default=5.0, help="segundos entre enable e disable collection")
    args = parser.parse_args()

    rclpy.init()
    node = Node("test_fleet_cases")

    def call_srv(srv_type, name: str, request, timeout_sec: float = 5.0):
        cli = node.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=timeout_sec):
            node.get_logger().error(f"Service {name} não disponível.")
            return None, False
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if not future.done():
            return None, False
        try:
            return future.result(), True
        except Exception as e:
            node.get_logger().error(f"Chamada falhou: {e}")
            return None, False

    ok_count = 0
    fail_count = 0

    # --- list_robots ---
    print("\n[1] list_robots")
    resp, ok = call_srv(ListRobots, "list_robots", ListRobots.Request())
    if ok and resp:
        print(f"    robot_ids: {list(resp.robot_ids)}")
        ok_count += 1
    else:
        print("    FALHOU (service não disponível ou sem resposta)")
        fail_count += 1

    # --- list_routes ---
    print(f"\n[2] list_routes(robot_id={args.robot})")
    req = ListRoutes.Request()
    req.robot_id = args.robot
    resp, ok = call_srv(ListRoutes, "list_routes", req)
    if ok and resp is not None:
        print(f"    route_names: {list(resp.route_names)}")
        ok_count += 1
    else:
        print("    FALHOU")
        fail_count += 1

    # --- start_record -> wait -> stop_record ---
    if not args.no_record:
        print(f"\n[3] start_record(robot_id={args.robot}, route_name={args.route})")
        req = StartRecord.Request()
        req.robot_id = args.robot
        req.route_name = args.route
        resp, ok = call_srv(StartRecord, "start_record", req)
        if ok and resp and resp.success:
            print(f"    OK: {resp.message}")
            ok_count += 1
        else:
            print(f"    FALHOU: {resp.message if resp else 'sem resposta'}")
            fail_count += 1

        time.sleep(args.wait_record)

        print(f"\n[4] stop_record(robot_id={args.robot})")
        req = StopRecord.Request()
        req.robot_id = args.robot
        resp, ok = call_srv(StopRecord, "stop_record", req)
        if ok and resp and resp.success:
            # Evitar falso positivo: rota vazia = TF provavelmente inexistente
            if "Route length: 0" in resp.message or "Route length:0" in resp.message:
                print(f"    FALHOU: rota vazia (confira TF: map -> {args.robot}/base_link?)")
                fail_count += 1
            else:
                print(f"    OK: {resp.message}")
                ok_count += 1
                # Conferir que a rota foi salva e aparece em list_routes
                req_r = ListRoutes.Request()
                req_r.robot_id = args.robot
                resp_r, ok_r = call_srv(ListRoutes, "list_routes", req_r)
                if ok_r and resp_r and args.route not in list(resp_r.route_names):
                    print(f"    FALHOU: rota '{args.route}' não está em list_routes (arquivo não criado?)")
                    fail_count += 1
        else:
            print(f"    FALHOU: {resp.message if resp else 'sem resposta'}")
            fail_count += 1
    else:
        print("\n[3-4] record pulado (--no-record)")

    # --- enable_collection -> wait -> disable_collection ---
    if not args.no_collection:
        print(f"\n[5] enable_collection(robot_id={args.robot}, topics=[scan, odom], rosbag2)")
        req = EnableCollection.Request()
        req.robot_id = args.robot
        req.topics = ["scan", "odom"]
        req.output_mode = "rosbag2"
        resp, ok = call_srv(EnableCollection, "enable_collection", req)
        if ok and resp and resp.success:
            print(f"    OK: {resp.message}")
            ok_count += 1
        else:
            print(f"    FALHOU: {resp.message if resp else 'sem resposta'}")
            fail_count += 1

        time.sleep(args.wait_collection)

        print(f"\n[6] disable_collection(robot_id={args.robot})")
        req = DisableCollection.Request()
        req.robot_id = args.robot
        resp, ok = call_srv(DisableCollection, "disable_collection", req)
        if ok and resp and resp.success:
            print(f"    OK: {resp.message}")
            ok_count += 1
        else:
            print(f"    FALHOU: {resp.message if resp else 'sem resposta'}")
            fail_count += 1
    else:
        print("\n[5-6] coleta pulada (--no-collection)")

    # --- collection_status ---
    print(f"\n[7] collection_status(robot_id={args.robot})")
    req = CollectionStatus.Request()
    req.robot_id = args.robot
    resp, ok = call_srv(CollectionStatus, "collection_status", req)
    if ok and resp is not None:
        print(f"    is_collecting={resp.is_collecting}, current_file={resp.current_file!r}, bytes={resp.bytes_written}")
        ok_count += 1
    else:
        print("    FALHOU")
        fail_count += 1

    # --- play_route (opcional) ---
    if args.with_play:
        print(f"\n[8] play_route(robot_id={args.robot}, route_name={args.route})")
        req = PlayRoute.Request()
        req.robot_id = args.robot
        req.route_name = args.route
        resp, ok = call_srv(PlayRoute, "play_route", req, timeout_sec=10.0)
        if ok and resp and resp.success:
            print(f"    OK: {resp.message}")
            ok_count += 1
        else:
            print(f"    FALHOU (Nav2/rota?): {resp.message if resp else 'sem resposta'}")
            fail_count += 1
    else:
        print("\n[8] play_route pulado (use --with-play para testar)")

    # --- resumo ---
    print("\n" + "=" * 50)
    print(f"Resumo: {ok_count} ok, {fail_count} falha(s)")
    print("=" * 50)

    node.destroy_node()
    rclpy.shutdown()
    return 0 if fail_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
