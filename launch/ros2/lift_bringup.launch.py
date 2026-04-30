#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    enable_bridge = DeclareLaunchArgument(
        'enable_bridge',
        default_value='false',
        description='Whether to enable the hex_bridge node, '
                    'you can set it to false if you want to launch the node separately.'
    )

    url = DeclareLaunchArgument(
        'url',
        default_value='0.0.0.0:8439',
        description='The URL of the robot.'
    )

    read_only = DeclareLaunchArgument(
        'read_only',
        default_value='false',
        description='Whether to read only the arm state.'
    )

    is_kcp = DeclareLaunchArgument(
        'is_kcp',
        default_value='true',
        description='Is KCP mode'
    )

    enable_ros_clock = DeclareLaunchArgument(
        'enable_ros_clock',
        default_value='true',
        description='Default to ROS clock source; use device internal clock if false.'
    )

    # ========== Lift group ==========
    # PushRosNamespace is commented out by default so topics stay in global
    # namespace. Uncomment to prefix topics with /lift/ (e.g. /lift/joint_cmd).

    lift_group = GroupAction([
        # PushRosNamespace('lift'),
        Node(
            package='hex_bridge',
            executable='hex_bridge',
            name='hex_bridge',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('enable_bridge')),
            parameters=[{
                'url': LaunchConfiguration('url'),
                'read_only': LaunchConfiguration('read_only'),
                'is_kcp': LaunchConfiguration('is_kcp'),
            }],
            remappings=[
                # subscribe
                ('/ws_down', 'ws_down'),
                # publish
                ('/ws_up', 'ws_up'),
            ],
        ),
        Node(
            package='hex_device_ros_wrapper',
            executable='lift_trans',
            name='lift_trans',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'enable_ros_clock': LaunchConfiguration('enable_ros_clock'),
            }],
            remappings=[
                # subscribe
                ('/xtopic_lift/joint_cmd', 'joint_cmd'),
                # publish
                ('/xtopic_lift/motor_states', 'motor_states'),
            ],
        ),
    ])

    return LaunchDescription([
        # parameters
        enable_bridge,
        url,
        read_only,
        is_kcp,
        enable_ros_clock,
        # group (contains bridge + trans nodes)
        lift_group,
    ])
