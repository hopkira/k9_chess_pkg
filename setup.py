from setuptools import setup
import os
from glob import glob

package_name = 'k9_chess'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include any scripts or BT files
        (os.path.join('share', package_name), glob('k9_chess/bt/*.py')),
        (os.path.join('share', package_name), glob('k9_chess/utils/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Richard Hopkins',
    author_email='richard@example.com',
    description='ROS 2 Python nodes for K9 chess robot (Lichess + Stockfish)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_manager_node = k9_chess.game_manager_node:main',
            'chess_engine_node = k9_chess.chess_engine_node:main',
            'move_sender_node = k9_chess.move_sender_node:main',
            'chess_state_node = k9_chess.chess_state_node:main',
            'chess_bt = k9_chess.bt.chess_bt:main',
        ],
    },
)