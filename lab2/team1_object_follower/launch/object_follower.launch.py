#!/usr/bin/env python3
# Launch both nodes together on the TurtleBot3
# Usage: ros2 launch team1_object_follower object_follower.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument('image_width',   default_value='320'),
        DeclareLaunchArgument('dead_band',      default_value='0.10'),
        DeclareLaunchArgument('angular_speed',  default_value='0.4'),
        DeclareLaunchArgument('max_angular',    default_value='0.8'),
        DeclareLaunchArgument('proportional',   default_value='True'),
        DeclareLaunchArgument('show_debug',     default_value='True'),
        DeclareLaunchArgument('min_area',       default_value='900.0'),
    ]

    find_object_node = Node(
        package='team1_object_follower',
        executable='find_object',
        name='find_object',
        output='screen',
        parameters=[{
            'min_area':   LaunchConfiguration('min_area'),
            'show_debug': LaunchConfiguration('show_debug'),
        }],
    )

    rotate_robot_node = Node(
        package='team1_object_follower',
        executable='rotate_robot',
        name='rotate_robot',
        output='screen',
        parameters=[{
            'image_width':   LaunchConfiguration('image_width'),
            'dead_band':     LaunchConfiguration('dead_band'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'max_angular':   LaunchConfiguration('max_angular'),
            'proportional':  LaunchConfiguration('proportional'),
        }],
    )

    return LaunchDescription(args + [find_object_node, rotate_robot_node])
