from setuptools import setup, find_packages

package_name = 'k9_chess'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'rclpy',
        'python-chess',
        'py_trees',
        'requests',
    ],

    zip_safe=True,
    maintainer='Richard Hopkins',
    maintainer_email='hopkira@gmail.com',
    description='ROS 2 Python nodes for K9 Chess Robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'game_manager_node = k9_chess.game_manager_node:main',
            'chess_engine_node = k9_chess.chess_engine_node:main',
            'chess_state_node = k9_chess.chess_state_node:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/k9_chess']),
        ('share/k9_chess/launch', ['launch/chess_launch.py']),
        ],
)

