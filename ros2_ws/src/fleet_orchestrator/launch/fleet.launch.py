#!/usr/bin/env python3
"""Launch do fleet: orchestrator + collector. Sim: --params-file config/single_robot_sim.yaml."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_shared_map_arg = DeclareLaunchArgument(
        "use_shared_map_frame",
        default_value="false",
        description="Frame 'map' sem prefixo. true para sim ou mapa único.",
    )
    routes_dir_arg = DeclareLaunchArgument("routes_dir", default_value="routes")
    collections_dir_arg = DeclareLaunchArgument("collections_dir", default_value="collections")

    use_shared = LaunchConfiguration("use_shared_map_frame")
    routes_dir = LaunchConfiguration("routes_dir")
    collections_dir = LaunchConfiguration("collections_dir")

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

    return LaunchDescription([
        use_shared_map_arg,
        routes_dir_arg,
        collections_dir_arg,
        orchestrator,
        collector,
    ])
