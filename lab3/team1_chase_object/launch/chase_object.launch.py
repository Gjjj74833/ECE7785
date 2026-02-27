#!/usr/bin/env python3
# Launch all three Lab 3 nodes
# Usage: ros2 launch team1_chase_object chase_object.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        # detect_object params
        DeclareLaunchArgument('min_area',         default_value='900.0'),
        DeclareLaunchArgument('show_debug',        default_value='True'),
        DeclareLaunchArgument('image_width',       default_value='320'),

        # get_object_range params
        DeclareLaunchArgument('lidar_window',      default_value='3'),

        # chase_object params
        DeclareLaunchArgument('desired_distance',  default_value='0.5'),
        DeclareLaunchArgument('ang_kp',            default_value='1.5'),
        DeclareLaunchArgument('ang_ki',            default_value='0.0'),
        DeclareLaunchArgument('ang_kd',            default_value='0.05'),
        DeclareLaunchArgument('lin_kp',            default_value='0.5'),
        DeclareLaunchArgument('lin_ki',            default_value='0.0'),
        DeclareLaunchArgument('lin_kd',            default_value='0.1'),
        DeclareLaunchArgument('max_angular',       default_value='1.0'),
        DeclareLaunchArgument('max_linear',        default_value='0.15'),
    ]

    detect_object_node = Node(
        package='team1_chase_object',
        executable='detect_object',
        name='detect_object',
        output='screen',
        parameters=[{
            'min_area':    LaunchConfiguration('min_area'),
            'show_debug':  LaunchConfiguration('show_debug'),
            'image_width': LaunchConfiguration('image_width'),
        }],
    )

    get_object_range_node = Node(
        package='team1_chase_object',
        executable='get_object_range',
        name='get_object_range',
        output='screen',
        parameters=[{
            'lidar_window': LaunchConfiguration('lidar_window'),
        }],
    )

    chase_object_node = Node(
        package='team1_chase_object',
        executable='chase_object',
        name='chase_object',
        output='screen',
        parameters=[{
            'desired_distance': LaunchConfiguration('desired_distance'),
            'ang_kp':           LaunchConfiguration('ang_kp'),
            'ang_ki':           LaunchConfiguration('ang_ki'),
            'ang_kd':           LaunchConfiguration('ang_kd'),
            'lin_kp':           LaunchConfiguration('lin_kp'),
            'lin_ki':           LaunchConfiguration('lin_ki'),
            'lin_kd':           LaunchConfiguration('lin_kd'),
            'max_angular':      LaunchConfiguration('max_angular'),
            'max_linear':       LaunchConfiguration('max_linear'),
        }],
    )

    return LaunchDescription(args + [
        detect_object_node,
        get_object_range_node,
        chase_object_node,
    ])
