import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)


def generate_launch_description():
    share = Path(get_package_share_directory("k9_chess_pkg"))
    config = str(share / "config" / "chess.yaml")

    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    'k9_chess_pkg'
                ),
                'launch',
                'phantom_board.launch.py',
            )
        )
    ),

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
