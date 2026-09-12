from glob import glob
import os

from setuptools import find_packages, setup


package_name = "k9_chess_pkg"

setup(
    name=package_name,
    version="0.2.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml", "README.md", "ARCHITECTURE.md", "VERSION"],
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hopkira",
    maintainer_email="hopkira@googlemail.com",
    description="K9 Lichess/Phantom chess manager and Stockfish action server.",
    license="Apache-2.0",
    scripts=[
        "scripts/chess_manager",
        "scripts/chess_engine",
        "scripts/chess_check",
    ],
    entry_points={
        'console_scripts': [
            'phantom_board = k9_chess_pkg.phantom_board_node:main',
        ],
    },
)
