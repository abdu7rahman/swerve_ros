"""Gamepad -> kinematics -> drive board, the full ROS 1 stack in one launch.

ROS 1 needed four terminals: roscore, joy_node, ps.py, swerve_controller.py,
plus two rosserial serial_node.py instances. This starts all of it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('swerve_ros2')
    params = os.path.join(pkg_share, 'config', 'swerve_params.yaml')

    use_agent = LaunchConfiguration('use_agent')
    serial_port = LaunchConfiguration('serial_port')
    baud = LaunchConfiguration('baud')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_agent', default_value='true',
            description='Start a micro-ROS agent for the drive board.'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            description='Serial device the drive board enumerates as.'),
        DeclareLaunchArgument(
            'baud', default_value='115200',
            description='micro-ROS serial transport baud rate.'),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[params],
        ),

        Node(
            package='swerve_ros2',
            executable='joy_teleop',
            name='joy_teleop',
            output='screen',
            parameters=[params],
        ),

        Node(
            package='swerve_ros2',
            executable='swerve_controller',
            name='swerve_controller',
            output='screen',
            parameters=[params],
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            condition=IfCondition(use_agent),
            arguments=['serial', '--dev', serial_port, '-b', baud],
        ),
    ])
