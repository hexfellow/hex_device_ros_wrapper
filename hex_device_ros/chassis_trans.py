#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
import time
import os
import sys

script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)

from ros_interface import DataInterface
from hex_device import HexDeviceApi, public_api_up_pb2
from hex_device import Chassis
from hex_device.motor_base import CommandType

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray, Bool


class HexChassisApi:
    """
    Hex Chassis API
    """

    def __init__(self):
        # 1. Create ROS interface
        self.ros_interface = DataInterface(name="xnode_chassis", rate_hz=500)
        self.ros_interface.set_rate(10)
        self._watchdog_check_every = max(1, int(self.ros_interface.get_rate() / 10.0))
        self._watchdog_counter = 0

        # 2. Get parameters
        self.ros_interface.set_parameter('frame_id', 'base_link')
        self.frame_id = self.ros_interface.get_parameter('frame_id')
        self.ros_interface.set_parameter('simple_mode', True)
        self.simple_mode = self.ros_interface.get_parameter('simple_mode')
        self.ros_interface.set_parameter('cmd_vel_timeout', 0.5)
        self._cmd_vel_timeout = float(self.ros_interface.get_parameter('cmd_vel_timeout') or 0.5)
        self._last_cmd_vel_time = 0.0  # 0 = never received, then use timeout to stop

        # 3. Create shared topics (ws_down, ws_up)
        self.ws_down_pub = self.ros_interface.create_publisher('ws_down', UInt8MultiArray, 10)
        self.ws_up_sub = self.ros_interface.create_subscription(
            'ws_up', UInt8MultiArray, self._ws_up_callback, 10)

        self.chassis = None
        self.motor_states_pub = None
        self.odom_pub = None
        self.joint_cmd_sub = None
        self.cmd_vel_sub = None
        self.clear_err_sub = None

        # 4. Init HexDeviceApi
        self.api = HexDeviceApi(control_hz=500, send_down_callback=self._pub_ws_down)
        self.version_check = False
        self.first_time = True

    # ========== Common topic callbacks ==========

    def _pub_ws_down(self, data):
        try:
            msg = UInt8MultiArray()
            msg.data = list(data)  # bytes -> list of uint8 for ROS2
            self.ros_interface.publish(self.ws_down_pub, msg)
        except Exception:
            pass

    def _ws_up_callback(self, msg):
        api_up = public_api_up_pb2.APIUp()
        try:
            api_up.ParseFromString(bytes(msg.data))
        except Exception:
            self.ros_interface.logw("Failed to parse ws_up message")
            return
        # 目前底盘暂不需要进行版本检查
        # if not self.version_check:
        #     self.version_check = True
        #     if not self.api._is_support_version(api_up):
        #         self.ros_interface.loge("Version mismatch: Chassis API is not supported")
        #         self.api.close()
        #         self.ros_interface.shutdown()
        self.api._process_api_up(api_up)

    # ========== Chassis-specific topic setup ==========

    def _setup_topics(self):
        self.motor_states_pub = self.ros_interface.create_publisher(
            '/xtopic_chassis/motor_states', JointState, 10)
        self.odom_pub = self.ros_interface.create_publisher(
            '/xtopic_chassis/odom', Odometry, 10)
        self.clear_err_sub = self.ros_interface.create_subscription(
            '/xtopic_chassis/clear_err', Bool, self._clear_err_callback, 10)

        if self.simple_mode:
            self.cmd_vel_sub = self.ros_interface.create_subscription(
                '/xtopic_chassis/cmd_vel', Twist, self._cmd_vel_callback, 10)
            self.ros_interface.logi("Chassis in simple mode, you can use cmd_vel topic to control chassis")
        else:
            self.joint_cmd_sub = self.ros_interface.create_subscription(
                '/xtopic_chassis/joint_cmd', JointState, self._joint_cmd_callback, 10)
            self.ros_interface.logi("Chassis in advanced mode, you can use joint_cmd topic to control chassis")

    # ========== Chassis-specific topic callbacks ==========

    def _get_chassis(self):
        if self.chassis is None:
            for device in self.api.device_list:
                if isinstance(device, Chassis):
                    self.chassis = device
                    return self.chassis
        return self.chassis

    def _joint_cmd_callback(self, msg):
        chassis = self._get_chassis()
        if chassis is not None:
            chassis.start()
            chassis.motor_command(CommandType.SPEED, msg.velocity)

    def _cmd_vel_callback(self, msg):
        try:
            self._last_cmd_vel_time = time.time()
            chassis = self._get_chassis()
            if chassis is not None:
                chassis.start()
                chassis.set_vehicle_speed(msg.linear.x, msg.linear.y, msg.angular.z)
        except Exception as e:
            print(f"Error in cmd_vel_callback: {e}")

    def _check_cmd_vel_timeout(self, chassis):
        """When no cmd_vel for timeout seconds, stop chassis (watchdog)."""
        if chassis.is_timeout():
            chassis.stop()

    def _clear_err_callback(self, msg):
        chassis = self._get_chassis()
        if chassis is not None and msg.data:
            chassis.clear_parking_stop()

    def _publish_odom(self, chassis):
        if self.odom_pub is None:
            return
        speeds = chassis.get_vehicle_speed(pop = False)
        if speeds is None:
            return
        position = chassis.get_vehicle_position(pop = False)
        if position is None:
            return
        speed_x, speed_y, speed_z = speeds
        x, y, yaw = position
        msg = Odometry()
        msg.header.stamp = self.ros_interface.get_timestamp()
        msg.header.frame_id = "odom"
        msg.child_frame_id = self.frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = speed_x
        msg.twist.twist.linear.y = speed_y
        msg.twist.twist.angular.z = speed_z
        self.ros_interface.publish(self.odom_pub, msg)

    def _publish_motor_states(self, chassis):
        if self.motor_states_pub is None:
            return
        motor_status = chassis.get_simple_motor_status()
        if motor_status is None:
            return
        msg = JointState()
        msg.header.stamp = self.ros_interface.get_timestamp_from_s_ns(motor_status['ts']['s'], motor_status['ts']['ns'])
        msg.name = [f"joint{i}" for i in range(len(motor_status['pos']))]
        msg.position = motor_status['pos'].tolist()
        msg.velocity = motor_status['vel'].tolist()
        msg.effort = motor_status['eff'].tolist()
        self.ros_interface.publish(self.motor_states_pub, msg)


# ========== Main Function ==========

def main():
    hex_chassis_api = HexChassisApi()

    try:
        while True:
            if hex_chassis_api.api.is_api_exit():
                print("Public API has exited.")
                break

            for device in hex_chassis_api.api.device_list:
                if isinstance(device, Chassis):
                    if device.has_new_data():
                        if hex_chassis_api.first_time:
                            hex_chassis_api.first_time = False
                            device.clear_odom_bias()
                            hex_chassis_api._setup_topics()
                            hex_chassis_api.ros_interface.logi("Chassis initialized successfully")

                        hex_chassis_api._publish_odom(device)
                        hex_chassis_api._publish_motor_states(device)
                    hex_chassis_api._watchdog_counter += 1
                    if hex_chassis_api._watchdog_counter >= hex_chassis_api._watchdog_check_every:
                        hex_chassis_api._watchdog_counter = 0
                        hex_chassis_api._check_cmd_vel_timeout(device)

            hex_chassis_api.ros_interface.sleep()

    except KeyboardInterrupt:
        print("Received Ctrl-C.")
        hex_chassis_api.api.close()
    finally:
        pass

    print("Resources have been cleaned up.")
    exit(0)


if __name__ == '__main__':
    main()
