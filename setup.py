import os
from setuptools import setup, find_packages
from glob import glob

package_name = 'k9_chess'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),

    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        ('share/' + package_name + '/assets', [
            'assets/stockfish',
            'assets/Titans.bin'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hopkira',
    maintainer_email='hopkira@todo.todo',
    description='K9 Chess Package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'game_manager = k9_chess.game_manager_node:main',
            'move_sender = k9_chess.move_sender_node:main',
            'chess_engine = k9_chess.chess_engine_node:main',
            'chess_state = k9_chess.chess_state_node:main',
        ],
    },
)
