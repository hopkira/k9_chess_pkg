"""ROS 2 launch description for the Phantom Chessboard adapter node.

Runtime parameters are loaded from the package-installed YAML configuration so
BLE deployment settings can be changed without editing Python source.
"""

import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for ``k9_chess_pkg/phantom_board``."""
    package_share = (
        get_package_share_directory(
            "k9_chess_pkg"
        )
    )

    config = os.path.join(
        package_share,
        "config",
        "phantom_board.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="k9_chess_pkg",
                executable="phantom_board",
                name="phantom_board",
                output="screen",
                parameters=[config],
            )
        ]
    )
