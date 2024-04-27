#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='deltarobot_inputs',
            executable='gui_node'
        ),
        Node(
            package='deltarobot',
            executable='robot_controller_node'
        ),
        Node(
            package='deltarobot_utils',
            executable='gepetto_visualizer_node'
        ),
        Node(
            package='deltarobot',
            executable='task_manager_node'
        )
    ])