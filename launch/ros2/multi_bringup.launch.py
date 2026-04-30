#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

# The template for multi launch like chassis and lift.

def generate_launch_description():
    # ========== Chassis arguments ==========

    enable_chassis_bridge = DeclareLaunchArgument(
        'enable_chassis_bridge',
        default_value='false',
        description='Whether to enable the hex_bridge node for chassis. '
                    'Set to false if you want to launch it separately.'
    )

    chassis_url = DeclareLaunchArgument(
        'chassis_url',
        default_value='0.0.0.0:8439',
        description='The URL of the chassis.'
    )

    chassis_read_only = DeclareLaunchArgument(
        'chassis_read_only',
        default_value='false',
        description='Whether to read only the chassis state.'
    )

    chassis_is_kcp = DeclareLaunchArgument(
        'chassis_is_kcp',
        default_value='true',
        description='Is KCP mode for chassis.'
    )

    frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='base_link',
        description='Frame ID of the chassis.'
    )

    simple_mode = DeclareLaunchArgument(
        'simple_mode',
        default_value='true',
        description='Simple mode of the chassis.'
    )

    # ========== Lift arguments ==========

    enable_lift_bridge = DeclareLaunchArgument(
        'enable_lift_bridge',
        default_value='false',
        description='Whether to enable the hex_bridge node for lift. '
                    'Set to false if you want to launch it separately.'
    )

    lift_url = DeclareLaunchArgument(
        'lift_url',
        default_value='0.0.0.0:8439',
        description='The URL of the lift.'
    )

    lift_read_only = DeclareLaunchArgument(
        'lift_read_only',
        default_value='false',
        description='Whether to read only the lift state.'
    )

    lift_is_kcp = DeclareLaunchArgument(
        'lift_is_kcp',
        default_value='true',
        description='Is KCP mode for lift.'
    )

    # ========== Shared arguments ==========

    enable_ros_clock = DeclareLaunchArgument(
        'enable_ros_clock',
        default_value='true',
        description='Default to ROS clock source; use device internal clock if false.'
    )

    # ========== Chassis group (namespace: /chassis) ==========

    chassis_group = GroupAction([
        PushRosNamespace('chassis'),
        # chassis bridge
        Node(
            package='hex_bridge',
            executable='hex_bridge',
            name='hex_bridge',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('enable_chassis_bridge')),
            parameters=[{
                'url': LaunchConfiguration('chassis_url'),
                'read_only': LaunchConfiguration('chassis_read_only'),
                'is_kcp': LaunchConfiguration('chassis_is_kcp'),
            }],
            remappings=[
                ('/ws_down', 'ws_down'),
                ('/ws_up', 'ws_up'),
            ],
        ),
        # chassis trans node
        Node(
            package='hex_device_ros_wrapper',
            executable='chassis_trans',
            name='hex_chassis',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'simple_mode': LaunchConfiguration('simple_mode'),
                'enable_ros_clock': LaunchConfiguration('enable_ros_clock'),
            }],
            remappings=[
                # subscribe
                ('/xtopic_chassis/motor_states', 'motor_states'),
                ('/xtopic_chassis/odom', 'odom'),
                # publish
                ('/xtopic_chassis/joint_cmd', 'joint_cmd'),
                ('/xtopic_chassis/cmd_vel', 'cmd_vel'),
                ('/xtopic_chassis/clear_err', 'clear_err'),
            ],
        ),
    ])

    # ========== Lift group (namespace: /lift) ==========

    lift_group = GroupAction([
        PushRosNamespace('lift'),
        # lift bridge
        Node(
            package='hex_bridge',
            executable='hex_bridge',
            name='hex_bridge',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('enable_lift_bridge')),
            parameters=[{
                'url': LaunchConfiguration('lift_url'),
                'read_only': LaunchConfiguration('lift_read_only'),
                'is_kcp': LaunchConfiguration('lift_is_kcp'),
            }],
            remappings=[
                ('/ws_down', 'ws_down'),
                ('/ws_up', 'ws_up'),
            ],
        ),
        # lift trans node
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
        # chassis arguments
        enable_chassis_bridge,
        chassis_url,
        chassis_read_only,
        chassis_is_kcp,
        frame_id,
        simple_mode,
        # lift arguments
        enable_lift_bridge,
        lift_url,
        lift_read_only,
        lift_is_kcp,
        # shared arguments
        enable_ros_clock,
        # device groups
        chassis_group,
        lift_group,
    ])
