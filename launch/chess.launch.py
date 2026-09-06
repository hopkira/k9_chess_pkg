from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("k9_chess_pkg"))
    config = str(share / "config" / "chess.yaml")

    return LaunchDescription(
        [
            Node(
                package="k9_chess_pkg",
                executable="chess_engine",
                name="chess_engine",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="k9_chess_pkg",
                executable="chess_manager",
                name="chess_manager",
                output="screen",
                parameters=[config],
            ),
        ]
    )
