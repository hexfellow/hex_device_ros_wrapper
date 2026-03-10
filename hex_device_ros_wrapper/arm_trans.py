#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-3-3
################################################################

import sys
import numpy as np
import time
import os
import signal
import sys

script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)

from ros_interface import DataInterface
import hex_device
from hex_device import HexDeviceApi, public_api_up_pb2
from hex_device.motor_base import CommandType

from std_msgs.msg import UInt8MultiArray
from hex_device_msgs.msg import XmsgArmJointParamList
from sensor_msgs.msg import JointState

class HexArmApi:
    """
    Hex Arm API
    """

    def __init__(self):
        # 1. Create ROS interface
        self.ros_interface = DataInterface(name="xnode_arm", rate_hz=500)
        # Timeout check every 10Hz
        self._watchdog_check_every = max(1, int(0.1 * self.ros_interface.get_rate()))
        self._watchdog_counter = 0
        self._last_arm_timeout_log = 0.0

        # 2. Get parameters
        self.ros_interface.set_parameter('arm_series', 0)
        self.arm_series = self.ros_interface.get_parameter('arm_series')
        if self.arm_series == 0:
            self.ros_interface.loge(f"Arm series is not set. Please checkout your config!")
            self.ros_interface.shutdown()
            exit(1)
        self.ros_interface.set_parameter('gripper_type', 0)
        self.gripper_type = self.ros_interface.get_parameter('gripper_type')
        self.ros_interface.set_parameter('joint_config_path', '')
        self.joint_config_path = self.ros_interface.get_parameter('joint_config_path')
        self.ros_interface.set_parameter('init_pose_path', '')
        init_pose_path = self.ros_interface.get_parameter('init_pose_path')
        # load auto init position
        if init_pose_path is not None:
            self.init_pos = self.ros_interface.get_init_pos_config(init_pose_path)

        self.version_check = False
        self.arm = None
        self.hands = None

        # 3. Create shared topics (ws_down, ws_up)
        self.ws_down_pub = self.ros_interface.create_publisher('ws_down', UInt8MultiArray, 10)
        self.ws_up_sub = self.ros_interface.create_subscription(
            'ws_up', UInt8MultiArray, self._ws_up_callback, 10)
        # 4. Init HexDeviceApi
        self.api = HexDeviceApi(control_hz=500, send_down_callback=self._pub_ws_down)
        hex_device.set_log_level("WARNING")

    def _check_cmd_timeout(self, arm):
        """When no cmd_vel for timeout seconds, stop chassis (watchdog)."""
        if arm.is_timeout():
            now = time.monotonic()
            if now - self._last_arm_timeout_log >= 5.0:
                self._last_arm_timeout_log = now
                self.ros_interface.logw("Arm command timeout, stopping arm...")
            arm.stop()

    def set_devices(self, arm, hands=None):
        """Set arm and hands device references (from main) and create arm/hands topics."""
        self.arm = arm[0] if isinstance(arm, list) and len(arm) > 0 else arm
        self.hands = hands[0] if isinstance(hands, list) and len(hands) > 0 else hands
        self.ros_interface.logi(f"Arm device: {self.arm}, Hands device: {self.hands}")
        self.joint_cmd_sub = self.ros_interface.create_subscription(
            '/xtopic_arm/joints_cmd',
            XmsgArmJointParamList,
            self._joint_cmd_callback,
            10
        )
        self.joint_states_pub = self.ros_interface.create_publisher(
            '/xtopic_arm/joint_states',
            JointState,
            10
        )
        if self.gripper_type != 0 and hands is not None:
            self.gripper_cmd_sub = self.ros_interface.create_subscription(
                '/xtopic_arm/gripper_cmd',
                XmsgArmJointParamList,
                self._gripper_cmd_callback,
                10
            )
            self.gripper_states_pub = self.ros_interface.create_publisher(
                '/xtopic_arm/gripper_states',
                JointState,
                10
            )

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
        if not self.version_check:
            self.version_check = True
            if not self.api._is_support_version(api_up):
                self.ros_interface.loge("Version mismatch, API closed")
                self.api.close()
                self.ros_interface.shutdown()
                return
        self.api._process_api_up(api_up)

    # ========== Arm-specific topic callbacks ==========
    def _joint_cmd_callback(self, msg):
        """Process arm's joint commands"""
        if self.arm is not None:
            self.arm.start()
            self.ros_interface.process_motor_command(msg, self.arm, 'arm')

    def _publish_joint_states(self):
        """Publish arm's joint states"""
        if self.arm is not None:
            motor_status = self.arm.get_simple_motor_status(pop=False)
            if motor_status is None:
                return
            try:
                msg = JointState()
                msg.header.stamp = self.ros_interface.get_timestamp_from_s_ns(motor_status['ts']['s'], motor_status['ts']['ns'])
                msg.name = [f"joint{i}" for i in range(len(motor_status['pos']))]
                msg.position = motor_status['pos'].tolist()
                msg.velocity = motor_status['vel'].tolist()
                msg.effort = motor_status['eff'].tolist()
                self.ros_interface.publish(self.joint_states_pub, msg)
            except Exception as e:
                self.ros_interface.loge(f"Error publishing joint states: {e}")

    # ========== Hands-specific topic callbacks ==========
    def _gripper_cmd_callback(self, msg):
        """Process gripper commands"""
        if self.hands is not None:
            self.ros_interface.process_motor_command(msg, self.hands, 'gripper')

    def _publish_gripper_states(self):
        """Publish gripper states"""
        if self.hands is not None:
            motor_status = self.hands.get_simple_motor_status(pop=False)
            if motor_status is None:
                return
            try:
                msg = JointState()
                msg.header.stamp = self.ros_interface.get_timestamp_from_s_ns(motor_status['ts']['s'], motor_status['ts']['ns'])
                msg.name = [f"gripper{i}" for i in range(len(motor_status['pos']))]
                msg.position = motor_status['pos'].tolist()
                msg.velocity = motor_status['vel'].tolist()
                msg.effort = motor_status['eff'].tolist()
                self.ros_interface.publish(self.gripper_states_pub, msg)
            except Exception as e:
                self.ros_interface.loge(f"Error publishing gripper states: {e}")

def signal_handler(signum, frame, hex_arm_api):
    """Custom signal handler for graceful shutdown"""
    # stop arm
    if hex_arm_api.arm is not None:
        hex_arm_api.arm.stop()
        time.sleep(1.0)

    print("[Shutdown] Shutting down ROS interface...")
    hex_arm_api.ros_interface.shutdown()
    print("[Shutdown] Complete")
    
def main():
    # Init HexDeviceApi
    hex_arm_api = HexArmApi()

    # Find device and calibrate
    hex_arm = None
    hex_hands = None
    while hex_arm_api.ros_interface.ok():
        need_arm = hex_arm is None
        need_hands = (hex_arm_api.gripper_type != 0 and hex_hands is None)
        if not need_arm and not need_hands:
            break
        if need_arm:
            raw = hex_arm_api.api.find_device_by_robot_type(hex_arm_api.arm_series)
            hex_arm = raw[0] if isinstance(raw, list) and len(raw) > 0 else raw
            if hex_arm is None:
                hex_arm_api.ros_interface.logi(f"Arm device not found, retry...")
                time.sleep(0.5)
                continue
        if need_hands:
            raw = hex_arm_api.api.find_optional_device_by_robot_type(hex_arm_api.gripper_type)
            hex_hands = raw[0] if isinstance(raw, list) and len(raw) > 0 else raw
            if hex_hands is None:
                hex_arm_api.ros_interface.logi(f"Gripper device not found, retry...")
                time.sleep(0.5)
    if hex_arm is None:
        hex_arm_api.ros_interface.shutdown()
        exit(1)

    # reload arm config
    if hex_arm_api.joint_config_path is not None:
        joint_config = hex_arm_api.ros_interface.get_config_from_json(hex_arm_api.joint_config_path)
        if joint_config is not None:
            hex_arm.reload_arm_config_from_dict(joint_config)
        else:
            hex_arm_api.ros_interface.loge(f"Joint config is None,using default config.")

    # Set device refs and create arm/hands topics (callbacks use hex_arm_api.arm / hex_arm_api.hands)
    hex_arm_api.set_devices(hex_arm, hex_hands)

    signal.signal(signal.SIGINT, lambda signum, frame: signal_handler(signum, frame, hex_arm_api))
    signal.signal(signal.SIGTERM, lambda signum, frame: signal_handler(signum, frame, hex_arm_api))

    # Init position
    if hex_arm_api.init_pos is not None:
        init_pos = hex_arm_api.init_pos['init_pos']
        step_limits = hex_arm_api.init_pos['step_limits']
        hex_arm_api.ros_interface.logi(f"Init position: {init_pos}")
        hex_arm_api.ros_interface.logi(f"Step limits: {step_limits}")
        if len(init_pos) != len(hex_arm):
            hex_arm_api.ros_interface.loge(f"Init position length mismatch, skill arm init.")
        else:
            # begin move to init position
            move_sucess = False
            while hex_arm_api.ros_interface.ok() and not hex_arm_api.api.is_api_exit():
                hex_arm.start()
                current_pos = hex_arm.get_motor_positions()
                if current_pos is None:
                    continue
                current_pos = np.array(current_pos)
                target_pos = np.array(init_pos)
                err = init_pos - current_pos

                # Send command to arm
                if step_limits is not None:
                    step_limits = np.array(step_limits)
                    err = np.clip(err, -step_limits, step_limits)
                next_pos = current_pos + err
                hex_arm.motor_command(CommandType.POSITION, next_pos.tolist())

                # Check if init pose is reached (within tolerance)
                if np.allclose(current_pos, target_pos, atol=0.05):
                    hex_arm_api.ros_interface.logi("Init pose reached.")
                    move_sucess = True
                    break
    
        if move_sucess == False:
            # exit with interrupt
            hex_arm_api.ros_interface.loge("Move to init position failed")
            hex_arm_api.api.close()
            hex_arm_api.ros_interface.shutdown()
            exit(1)

    # main loop
    while hex_arm_api.ros_interface.ok() and not hex_arm_api.api.is_api_exit():
        try:
            hex_arm_api.ros_interface.sleep()
        except Exception as e:
            if "rclpy.shutdown() has been called" not in str(e):
                hex_arm_api.ros_interface.loge(f"Error sleeping: {e}")
            exit(1)
        # publish joint states
        hex_arm_api._publish_joint_states()
        # publish gripper states (no-op if hex_arm_api.hands is None)
        hex_arm_api._publish_gripper_states()

        # check if cmd is timeout
        hex_arm_api._watchdog_counter += 1
        if hex_arm_api._watchdog_counter >= hex_arm_api._watchdog_check_every:
            hex_arm_api._watchdog_counter = 0
            hex_arm_api._check_cmd_timeout(hex_arm_api.arm)

if __name__ == "__main__":
    main()
