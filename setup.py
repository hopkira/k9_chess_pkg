from setuptools import setup, find_packages

package_name = 'k9_chess'

setup(
    name=package_name,
    version='0.1.0',

    packages= find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'assets'), glob('assets/*')),
        (os.path.join('share', package_name, 'bt'), glob('bt/*')),
    ],
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
            'game_manager = k9_chess.game_manager_node:main',
            'chess_engine = k9_chess.chess_engine_node:main',
            'chess_state = k9_chess.chess_state_node:main',
        ],
    },
)

