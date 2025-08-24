# k9_chess/launch/chess_launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Game Manager Node
        Node(
            package='k9_chess',
            executable='game_manager',
            name='game_manager',
            output='screen',
            parameters=[],
        ),

        # Chess Engine Node
        Node(
            package='k9_chess',
            executable='chess_engine',
            name='chess_engine',
            output='screen',
            parameters=[],
        ),

        # Move Sender Node
        Node(
            package='k9_chess',
            executable='move_sender',
            name='move_sender',
            output='screen',
            parameters=[],
        ),

        # Chess State Node
        Node(
            package='k9_chess',
            executable='chess_state',
            name='chess_state',
            output='screen',
            parameters=[],
        ),

        # Behavior Tree Orchestrator
        Node(
            package='k9_chess',
            executable='chess_bt',
            name='chess_bt',
            output='screen',
            parameters=[],
        ),
    ])