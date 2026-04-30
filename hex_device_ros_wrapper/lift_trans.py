#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
import time
import os
import sys

script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)

from ros_interface import DataInterface
from hex_device import HexDeviceApi, public_api_up_pb2, LinearLift
from hex_device import CommandType

from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray, Bool

class HexLiftApi:
    
    def __init__(self):
        # 1. Create ROS interface
        self.ros_interface = DataInterface(name="xnode_lift", rate_hz=500)
        # Timeout check every 10Hz
        self._watchdog_check_every = max(1, int(0.1 * self.ros_interface.get_rate()))
        self._watchdog_counter = 0
        self._last_lift_timeout_log = 0.0

        # 2. Get parameters
        self.ros_interface.set_parameter('enable_ros_clock', True)
        self.enable_ros_clock = self.ros_interface.get_parameter('enable_ros_clock')

        self.version_check = False
        self.first_time = True
        
        # 3. Init HexDeviceApi
        self.api = HexDeviceApi(control_hz=500, send_down_callback=self._pub_ws_down)

        # 4. Create shared topics (ws_down, ws_up)
        self.ws_down_pub = self.ros_interface.create_publisher('ws_down', UInt8MultiArray, 10)
        self.ws_up_sub = self.ros_interface.create_subscription(
            'ws_up', UInt8MultiArray, self._ws_up_callback, 10)

        self.lift = None
        self.motor_states_pub = None
        self.joint_cmd_sub = None

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

    # ========== lift topic setup ==========

    def _setup_topics(self):
        self.motor_states_pub = self.ros_interface.create_publisher(
            '/xtopic_lift/motor_states', JointState, 10)

        self.joint_cmd_sub = self.ros_interface.create_subscription(
                '/xtopic_lift/joint_cmd', JointState, self._joint_cmd_callback, 10)
        
    # ========== lift topic callbacks ==========
    
    def _joint_cmd_callback(self, msg):
        lift = self._get_Lift()
        if lift is not None:
            lift.motor_command(CommandType.POSITION, msg.position[0])
        
    def _get_Lift(self):
        if self.lift is None:
            for device in self.api.device_list:
                if isinstance(device, LinearLift):
                    self.lift = device
                    return self.lift
        return self.lift

    def _publish_motor_states(self, lift):
        if self.motor_states_pub is None:
            return
        timestamp = self._get_clock_timestamp()
        position_status = lift.get_motor_positions()
        velocity_status = lift.get_move_speed() / lift._pulse_per_rotation
        if position_status is None:
            return
        msg = JointState()
        msg.header.stamp = timestamp
        msg.name = [f"joint1"]
        msg.position = [position_status]
        msg.velocity = [velocity_status]
        msg.effort = [0.0]
        self.ros_interface.publish(self.motor_states_pub, msg)
        
    # ========== tool ==========
        
    def _get_clock_timestamp(self):
        _timestamp = None
        
        device = self._get_Lift()

        if self.enable_ros_clock == True:
            _timestamp = self.ros_interface.get_timestamp()
        elif self.enable_ros_clock == False:
            _timestamp = self.ros_interface.get_timestamp_from_s_ns(device._last_update_time.s, device._last_update_time.ns)
        
        return _timestamp

# ========== Main Function ==========

def main():
    hex_Lift_api = HexLiftApi()
    
    # Wait for Lift device to appear (up to 30 seconds)
    Lift = None
    startup_timeout = 30.0
    startup_start = time.time()
    last_log_time = 0.0

    while hex_Lift_api.ros_interface.ok() and not hex_Lift_api.api.is_api_exit():
        Lift_list = hex_Lift_api.api.device_list
        for device in Lift_list:
            if isinstance(device, LinearLift):
                Lift = device
                break

        if Lift is not None:
            hex_Lift_api.ros_interface.logi("Lift device discovered")
            break

        elapsed = time.time() - startup_start
        if elapsed > startup_timeout:
            hex_Lift_api.ros_interface.loge(
                f"No Lift device detected within {startup_timeout:.0f}s. "
                "Check that hex_bridge is running and the robot is connected."
            )
            hex_Lift_api.api.close()
            hex_Lift_api.ros_interface.shutdown()
            exit(1)

        # Log waiting status every 3 seconds
        if elapsed - last_log_time >= 3.0:
            last_log_time = elapsed
            hex_Lift_api.ros_interface.logi(
                f"Waiting for Lift device... ({elapsed:.0f}s elapsed)"
            )

        time.sleep(0.5)

    try:
        while True:
            if hex_Lift_api.api.is_api_exit():
                print("Public API has exited.")
                break

            for device in hex_Lift_api.api.device_list:
                if isinstance(device, LinearLift):
                    if device.has_new_data():
                        if hex_Lift_api.first_time:
                            hex_Lift_api.first_time = False
                            hex_Lift_api._setup_topics()
                            device.start()
                            hex_Lift_api.ros_interface.logi("Lift initialized successfully")

                        hex_Lift_api._publish_motor_states(device)

            hex_Lift_api.ros_interface.sleep()

    except KeyboardInterrupt:
        print("Received Ctrl-C.")
        hex_Lift_api.api.close()
    finally:
        pass

    print("Resources have been cleaned up.")
    exit(0)


if __name__ == '__main__':
    main()
