#!/usr/bin/env python3
"""Launch do fleet: orchestrator + collector.

Simulação 1 robô (map sem prefixo): ``single_robot_sim:=true`` (carrega ``config/single_robot_sim.yaml``).

Nota: ``ros2 launch`` **não** aceita ``--params-file`` como o ``ros2 run``; use o argumento abaixo.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def launch_setup(context, *args, **kwargs):
    single = context.launch_configurations.get("single_robot_sim", "false").lower() in (
        "true",
        "1",
        "yes",
    )
    pkg = get_package_share_directory("fleet_orchestrator")
    single_yaml = os.path.join(pkg, "config", "single_robot_sim.yaml")

    use_shared = LaunchConfiguration("use_shared_map_frame")
    routes_dir = LaunchConfiguration("routes_dir")
    collections_dir = LaunchConfiguration("collections_dir")

    if single:
        orchestrator = Node(
            package="fleet_orchestrator",
            executable="fleet_orchestrator",
            name="fleet_orchestrator",
            output="screen",
            parameters=[ParameterFile(single_yaml, allow_substs=True)],
        )
        collector = Node(
            package="fleet_data_collector",
            executable="sensor_collector",
            name="sensor_collector",
            output="screen",
            parameters=[ParameterFile(single_yaml, allow_substs=True)],
        )
    else:
        orchestrator = Node(
            package="fleet_orchestrator",
            executable="fleet_orchestrator",
            name="fleet_orchestrator",
            output="screen",
            parameters=[
                {"routes_dir": routes_dir, "use_shared_map_frame": use_shared},
                {"robots": ["tb1", "tb2", "tb3"]},
            ],
        )
        collector = Node(
            package="fleet_data_collector",
            executable="sensor_collector",
            name="sensor_collector",
            output="screen",
            parameters=[
                {"collections_dir": collections_dir},
                {"robots": ["tb1", "tb2", "tb3"]},
            ],
        )
    return [orchestrator, collector]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "single_robot_sim",
            default_value="false",
            description="Se true, carrega config/single_robot_sim.yaml (robots [''], map partilhado).",
        ),
        DeclareLaunchArgument(
            "use_shared_map_frame",
            default_value="false",
            description="Frame 'map' sem prefixo. true para sim ou mapa único (modo multi).",
        ),
        DeclareLaunchArgument("routes_dir", default_value="routes"),
        DeclareLaunchArgument("collections_dir", default_value="collections"),
        OpaqueFunction(function=launch_setup),
    ])
