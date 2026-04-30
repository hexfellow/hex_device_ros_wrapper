#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    # Note: ROS2 launch does not provide control over node shutdown order.
    # It is recommended to launch hex_bridge_node separately to ensure proper
    # command forwarding during the shutdown phase and prevent premature termination
    # of the bridge node that may cause shutdown commands to fail.
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

    joint_config = FindPackageShare('hex_device_ros_wrapper').find(
        'hex_device_ros_wrapper') + '/config/joints.json'
    joint_config_path = DeclareLaunchArgument(
        'joint_config_path',
        default_value=joint_config,
        description='The path to the joint config file.'
    )

    init_pose_file_path = FindPackageShare('hex_device_ros_wrapper').find(
        'hex_device_ros_wrapper') + '/config/init_pos.json'
    init_pose_path = DeclareLaunchArgument(
        'init_pose_path',
        default_value=init_pose_file_path,
        description='The path to the init pose file.'
    )

    gripper_type = DeclareLaunchArgument(
        'gripper_type',
        default_value='0',
        description='The type of the Gripper (integer).'
    )

    arm_series = DeclareLaunchArgument(
        'arm_series',
        default_value='0',
        description='The series of the HexArm (integer).'
    )

    enable_ros_clock = DeclareLaunchArgument(
        'enable_ros_clock',
        default_value='true',
        description='Default to ROS clock source; use device internal clock if false.'
    )

    # ========== Arm group ==========
    # PushRosNamespace is commented out by default so topics stay in global
    # namespace. Uncomment to prefix topics with /arm/ (e.g. /arm/joint_states).

    arm_group = GroupAction([
        # PushRosNamespace('arm'),
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
            executable='arm_trans',
            name='hex_arm',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'joint_config_path': LaunchConfiguration('joint_config_path'),
                'init_pose_path': LaunchConfiguration('init_pose_path'),
                'gripper_type': LaunchConfiguration('gripper_type'),
                'arm_series': LaunchConfiguration('arm_series'),
                'enable_ros_clock': LaunchConfiguration('enable_ros_clock'),
            }],
            remappings=[
                ('/xtopic_arm/joints_cmd', 'joints_cmd'),
                ('/xtopic_arm/joint_states', 'joint_states'),
                ('/xtopic_arm/gripper_cmd', 'gripper_cmd'),
                ('/xtopic_arm/gripper_states', 'gripper_states'),
            ],
        ),
    ])

    return LaunchDescription([
        # parameters
        enable_bridge,
        url,
        read_only,
        is_kcp,
        joint_config_path,
        init_pose_path,
        gripper_type,
        arm_series,
        enable_ros_clock,
        # group (contains bridge + trans nodes)
        arm_group,
    ])