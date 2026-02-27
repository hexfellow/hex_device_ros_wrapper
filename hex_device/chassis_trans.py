#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import threading
import asyncio
import os
import sys
import numpy as np
import time
import signal
from enum import Enum

script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)

from ros_interface import DataInterface
from hex_device_py import Chassis, CommandType, public_api_up_pb2, public_api_down_pb2, public_api_types_pb2, Timestamp
from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Bool

class InitState(Enum):
    PENDING = 0
    SUCCESS = 1
    FAILED = 2

class HexChassisApi:
    """
    Hex Chassis API
    """

    def __init__(self):
        self.ros_interface = DataInterface(name="xnode_chassis")

        self.ros_interface.set_parameter('frame_id', 'base_link')
        self.frame_id = self.ros_interface.get_parameter('frame_id')
        self.ros_interface.set_parameter('simple_mode', True)
        self.simple_mode = self.ros_interface.get_parameter('simple_mode')

        self.ws_down_pub = self.ros_interface.create_publisher('ws_down', UInt8MultiArray, 10)
        self.ws_up_sub = self.ros_interface.create_subscription(
            'ws_up', UInt8MultiArray, self._ws_up_callback, 10)
        
        self.chassis = None
        self.init_state = InitState.PENDING

        self.joint_cmd_sub = None
        self.cmd_vel_sub = None
        self.clear_err_sub = None
        self.motor_states_pub = None
        self.odom_pub = None

        self.first_time = True

    async def _pub_ws_down(self, data):
        try:
            msg = UInt8MultiArray()
            msg.data = data.SerializeToString()
            self.ros_interface.publish(self.ws_down_pub, msg)
        except Exception:
            pass

    def _ws_up_callback(self, msg):
        api_up = public_api_up_pb2.APIUp()
        api_up.ParseFromString(bytes(msg.data))

        if self.init_state == InitState.PENDING and api_up.HasField('base_status'):
            motor_count = len(api_up.base_status.motor_status)
            robot_type = api_up.robot_type

            if robot_type not in Chassis.SUPPORTED_ROBOT_TYPES:
              self.ros_interface.loge(f"Unsupported chassis type: {robot_type}")
              self.init_state = InitState.FAILED
              return
            
            try:
                self.chassis = Chassis(
                    motor_count=motor_count,
                    robot_type=robot_type,
                    send_message_callback=self._pub_ws_down
                )
                self.chassis._set_robot_type(robot_type)
                self.chassis._update(api_up, Timestamp.from_ns(time.perf_counter_ns()))
                self.chassis.clear_odom_bias()

                if self.simple_mode == True:
                    self.cmd_vel_sub = self.ros_interface.create_subscription(
                        '/xtopic_chassis/cmd_vel', 
                        Twist, 
                        self._cmd_vel_callback, 
                        10
                    )
                    self.ros_interface.logi("Chassis in simple mode, you can use cmd_vel topic to control chassis")
                else:
                    self.joint_cmd_sub = self.ros_interface.create_subscription(
                        '/xtopic_chassis/joint_cmd', 
                        JointState, 
                        self._joint_cmd_callback, 
                        10
                    )
                    self.ros_interface.logi("Chassis in advanced mode, you can use joint_cmd topic to control chassis")

                self.motor_states_pub = self.ros_interface.create_publisher(
                    '/xtopic_chassis/motor_states', 
                    JointState, 
                    10
                )

                self.clear_err_sub = self.ros_interface.create_subscription(
                    '/xtopic_chassis/clear_err', 
                    Bool, 
                    self._clear_err_callback, 
                    10
                )

                self.odom_pub = self.ros_interface.create_publisher(
                    '/xtopic_chassis/odom', 
                    Odometry, 
                    10
                )
            except Exception as e:
                self.ros_interface.loge(f"Failed to initialize chassis: {e}")
                self.init_state = InitState.FAILED
                return

            self.init_state = InitState.SUCCESS
            self.ros_interface.logi("Chassis initialized successfully")

        if self.chassis is not None and self.init_state == InitState.SUCCESS:
          if api_up.robot_type == self.chassis.robot_type:
              self.chassis._update(api_up, Timestamp.from_ns(time.perf_counter_ns()))
              self._publish_odom()
              self._publish_motor_states()

    def _joint_cmd_callback(self, msg):
        if self.chassis is not None:
            if self.first_time:
                self.first_time = False
                self.chassis.start()
            self.chassis.motor_command(CommandType.SPEED, msg.velocity)

    def _cmd_vel_callback(self, msg):
        if self.chassis is not None:
            if self.first_time:
                self.first_time = False
                self.chassis.start()
            self.chassis.set_vehicle_speed(msg.linear.x, msg.linear.y, msg.angular.z)

    def _clear_err_callback(self, msg):
        if self.chassis is not None and msg.data:
            self.chassis.clear_parking_stop()

    def _publish_odom(self):
        if self.chassis is not None and self.chassis.has_new_data():
            msg = Odometry()
            msg.header.stamp = self.ros_interface.get_timestamp()
            msg.header.frame_id = "odom"
            msg.child_frame_id = self.frame_id
            speeds= self.chassis.get_vehicle_speed()
            if speeds is None:
                return
            speed_x, speed_y, speed_z = speeds
            position = self.chassis.get_vehicle_position()
            if position is None:
                return
            x, y, yaw = position
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

    def _publish_motor_states(self):
        if self.chassis is not None and self.chassis.has_new_data():
            motor_status = self.chassis.get_simple_motor_status()
            if motor_status is None:
                return
            msg = JointState()
            msg.header.stamp = self.ros_interface.get_timestamp_from_s_ns(motor_status['ts']['s'], motor_status['ts']['ns'])
            msg.name = [f"joint{i}" for i in range(len(motor_status['pos']))]
            msg.position = motor_status['pos'].tolist()
            msg.velocity = motor_status['vel'].tolist()
            msg.effort = motor_status['eff'].tolist()
            self.ros_interface.publish(self.motor_states_pub, msg)

async def _run_with_cancellation(coro, stop_event, loop):
    """Wrapper to run coroutine and check for stop event"""
    task = asyncio.create_task(coro)
    try:
        # Run task but check stop event periodically
        while not task.done():
            if stop_event.is_set():
                task.cancel()
                break
            await asyncio.sleep(0.01)
        await task
    except asyncio.CancelledError:
        # Task was cancelled, this is expected
        pass
    except Exception as e:
        print(f"Task exception: {e}")

def run_async_in_thread(coro, stop_event):
    """Helper function to run async coroutine in a thread with a new event loop"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(_run_with_cancellation(coro, stop_event, loop))
    except Exception as e:
        print(f"Async thread exception: {e}")
    finally:
        # Cancel all remaining tasks
        try:
            pending = asyncio.all_tasks(loop)
            for task in pending:
                task.cancel()
            # Wait for all tasks to complete cancellation
            if pending:
                loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
        except Exception:
            pass
        finally:
            loop.close()

def signal_handler(signum, frame, stop_event, shutdown_event, chassis_thread, api):
    """Custom signal handler for graceful shutdown"""
    # stop chassis
    if api.chassis is not None:
        print("\n[Ctrl-C] Received shutdown signal")
        api.chassis.stop()
        time.sleep(0.5)

    # Signal threads to stop
    print("[Shutdown] Stopping async threads...")
    stop_event.set()
    
    # Wait for threads to finish with timeout
    if chassis_thread and chassis_thread.is_alive():
        chassis_thread.join(timeout=2.0)
        if chassis_thread.is_alive():
            print("[Warning] Chassis thread did not stop gracefully")
    
    print("[Shutdown] Shutting down ROS interface...")
    api.ros_interface.shutdown()
    print("[Shutdown] Complete")
    shutdown_event.set()

def cmd_timeout_checker(api, shutdown_event):
    while True:
        if api.chassis.is_timeout():
            api.ros_interface.logw("Command timeout detected")
            api.chassis.stop()
            time.sleep(0.5)
            api.ros_interface.shutdown()
            shutdown_event.set()
            break
        api.ros_interface.sleep()

 # ========== Main Function ==========

def main():
    api = HexChassisApi()
    stop_event = threading.Event()
    shutdown_event = threading.Event()

    # Wait for chassis initialization
    api.ros_interface.logi("Waiting for chassis API initialization...")
    timeout = 10.0 
    start_time = time.perf_counter()

    while api.init_state == InitState.PENDING:
        if not api.ros_interface.ok():
            api.ros_interface.loge("ROS shutdown during initialization")
            return

        # Check timeout
        elapsed = time.perf_counter() - start_time
        if elapsed > timeout:
            api.ros_interface.loge(f"Chassis API initialization timeout after {timeout}s")
            api.ros_interface.shutdown()
            return

        time.sleep(0.1)

    # Check if initialization succeeded
    if api.init_state == InitState.FAILED:
        api.ros_interface.loge("Chassis API initialization failed")
        api.ros_interface.shutdown()
        return

    chassis_thread = None
    if api.chassis is not None:
        chassis_thread = threading.Thread(
            target=run_async_in_thread,
            args=(api.chassis._periodic(), stop_event)
        )
        chassis_thread.daemon = True
        chassis_thread.start()
        api.ros_interface.logi("Chassis thread started successfully")
    else:
        api.ros_interface.loge("Chassis thread start failed.")
        api.ros_interface.shutdown()
        return
    
    # wait thread to start
    time.sleep(0.2)

    signal.signal(signal.SIGINT, lambda signum, frame: signal_handler(signum, frame, stop_event, shutdown_event, chassis_thread, api))
    signal.signal(signal.SIGTERM, lambda signum, frame: signal_handler(signum, frame, stop_event, shutdown_event, chassis_thread, api))

    api.ros_interface.set_rate(10)
    threading.Thread(
        target=cmd_timeout_checker,
        args=(api, shutdown_event),
        daemon=True
    ).start()

    shutdown_event.wait()

if __name__ == '__main__':
    main()
