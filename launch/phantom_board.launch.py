"""ROS 2 launch description for the Phantom Chessboard adapter node.

Runtime parameters are loaded from the package-installed YAML configuration so
BLE deployment settings can be changed without editing Python source.

The standalone Phantom Chessboard source and K9 virtual environment are added
to PYTHONPATH because the ROS console executable runs under the system Python.
"""

import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch_ros.actions import Node


K9_VENV_SITE_PACKAGES = os.path.expanduser(
    "~/k9_venv/lib/python3.12/site-packages"
)

PHANTOM_CHESSBOARD_SRC = os.path.expanduser(
    "~/phantom_chessboard/src"
)


def generate_launch_description():
    """Build the launch description for ``k9_chess_pkg/phantom_board``."""

    package_share = get_package_share_directory(
        "k9_chess_pkg"
    )

    config = os.path.join(
        package_share,
        "config",
        "phantom_board.yaml",
    )

    existing_pythonpath = os.environ.get(
        "PYTHONPATH",
        "",
    )

    python_paths = [
        PHANTOM_CHESSBOARD_SRC,
        K9_VENV_SITE_PACKAGES,
    ]

    if existing_pythonpath:
        python_paths.append(
            existing_pythonpath
        )

    pythonpath = os.pathsep.join(
        python_paths
    )

    return LaunchDescription(
        [
            Node(
                package="k9_chess_pkg",
                executable="phantom_board",
                name="phantom_board",
                output="screen",
                parameters=[config],
                additional_env={
                    "PYTHONPATH": pythonpath,
                },
            )
        ]
    )