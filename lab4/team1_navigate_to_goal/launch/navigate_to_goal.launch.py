#!/usr/bin/env python3
# Launch all Lab 4 nodes
# Usage: ros2 launch team1_navigate_to_goal navigate_to_goal.launch.py
# Yihan Liu

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument('max_obstacle_dist', default_value='0.5'),
        DeclareLaunchArgument('min_cluster_beams', default_value='3'),
    ]

    get_object_range_node = Node(
        package='team1_navigate_to_goal',
        executable='get_object_range',
        name='get_object_range',
        output='screen',
        parameters=[{
            'max_obstacle_dist': LaunchConfiguration('max_obstacle_dist'),
            'min_cluster_beams': LaunchConfiguration('min_cluster_beams'),
        }],
    )

    go_to_goal_node = Node(
        package='team1_navigate_to_goal',
        executable='go_to_goal',
        name='go_to_goal',
        output='screen',
    )

    return LaunchDescription(args + [
        get_object_range_node,
        go_to_goal_node,
    ])
