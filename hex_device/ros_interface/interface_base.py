#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
ROS interface abstract base class
"""

import json
from abc import ABC, abstractmethod
from typing import Callable
from typing import Optional, Tuple, List, Dict, Any, Union
from hex_device_py import Arm, Hands, CommandType, MitMotorCommand, MotorBase


class InterfaceBase(ABC):
    """
    ROS interface abstract base class
    """

    def __init__(self, name: str):
        self._name = name

    # ========== Topic management (generic) ==========

    @abstractmethod
    def create_publisher(self, topic_name: str, msg_type: type, queue_size: int = 10):
        """
        Create publisher

        Args:
            topic_name: Topic name
            msg_type: Message type
            queue_size: Queue size

        Returns:
            Publisher object
        """
        pass

    @abstractmethod
    def create_subscription(self, topic_name: str, msg_type: type,
                           callback: Callable, queue_size: int = 10):
        """
        Create subscription

        Args:
            topic_name: Topic name
            msg_type: Message type
            callback: Callback function (defined externally)
            queue_size: Queue size

        Returns:
            Subscription object
        """
        pass

    @abstractmethod
    def publish(self, publisher, msg):
        """
        Publish message

        Args:
            publisher: Publisher object (returned by create_publisher)
            msg: Message object
        """
        pass

    # ========== Timer ==========

    @abstractmethod
    def create_timer(self, interval_sec: float, callback: Callable[[], None]):
        """
        Create timer

        Args:
            interval_sec: Timer interval (seconds)
            callback: Callback function (no parameters, defined externally)

        Returns:
            Timer object

        Note:
            - User-provided callback does not need to receive any parameters
            - ROS1/ROS2 internally handle differences
        """
        pass

    # ========== Parameter server ==========

    @abstractmethod
    def get_parameter(self, name: str):
        """
        Get parameter

        Args:
            name: Parameter name
        """
        pass

    @abstractmethod
    def set_parameter(self, name: str, value):
        """
        Set parameter

        Args:
            name: Parameter name
            value: Parameter value
        """
        pass

    # ========== Rate control ==========

    @abstractmethod
    def set_rate(self, hz: float):
        """
        Set rate controller frequency

        Args:
            hz: Frequency (Hz)
        """
        pass

    @abstractmethod
    def sleep(self):
        """Sleep according to rate"""
        pass

    # ========== Lifecycle ==========

    @abstractmethod
    def ok(self) -> bool:
        """Check if ROS is running normally"""
        pass

    @abstractmethod
    def shutdown(self):
        """Shutdown ROS node"""
        pass

    # ========== Logging ==========

    @abstractmethod
    def logd(self, msg: str, *args, **kwargs):
        """Output debug log"""
        pass

    @abstractmethod
    def logi(self, msg: str, *args, **kwargs):
        """Output info log"""
        pass

    @abstractmethod
    def logw(self, msg: str, *args, **kwargs):
        """Output warn log"""
        pass

    @abstractmethod
    def loge(self, msg: str, *args, **kwargs):
        """Output error log"""
        pass

    @abstractmethod
    def logf(self, msg: str, *args, **kwargs):
        """Output fatal log"""
        pass

    # ========== Tools ==========

    @abstractmethod
    def get_timestamp(self):
        """Get current time stamp"""
        pass

    # ========== Hex Device common function ==========
    def parse_extra_param(self, extra_param_str):
        try:
            if extra_param_str == "":
                return {}
            extra_param_dict = json.loads(extra_param_str)
            return extra_param_dict
        except json.JSONDecodeError:
            self.loge(f"Error: {extra_param_str} is not a valid value.")
            return {}

    def process_motor_command(self, msg, device: Optional[MotorBase], device_name: str):
        """
        Generic function to process motor commands for arm or gripper.
        
        Args:
            msg: Joint command message (XmsgArmJointParamList)
            device: Motor device (self.arm or self.hands)
            device_name: Name for logging ('arm' or 'gripper')
        """
        if device is None:
            self.logw(f"{device_name.capitalize()} not initialized.")
            return
        
        motor_count = len(device)
        
        if not hasattr(msg, 'joints') or msg.joints is None:
            self.logw(f"XmsgArmJointParamList message has no joints for {device_name}.")
            return
        
        length = len(msg.joints)
        if length != motor_count:
            self.logw(f"XmsgArmJointParamList message length {length} not match {device_name} motor count {motor_count}.")
            return
        
        # Priority check: brake in extra_param
        try:
            extra_params = [self.parse_extra_param(joint.extra_param) if hasattr(joint, 'extra_param') and joint.extra_param else {} for joint in msg.joints]
            brakes = [param.get('brake', False) for param in extra_params]
            
            if any(brakes):
                brake_commands = [True] * motor_count
                device.motor_command(CommandType.BRAKE, brake_commands)
                self.logi(f"Brake command detected in {device_name} extra_param and sent to all motors")
                return
        except Exception as e:
            self.loge(f"Error parsing {device_name} extra_param for brake: {e}")
            return
        
        # Check if all joints have the same mode
        modes = [joint.mode.lower() if hasattr(joint, 'mode') and joint.mode else None for joint in msg.joints]
        if None in modes:
            self.logw(f"{device_name.capitalize()} joint command is missing mode field.")
            return
        
        # Check if all modes are the same
        unique_modes = set(modes)
        if len(unique_modes) > 1:
            self.logw(f"Different control modes detected in {device_name} joints: {list(unique_modes)}. All joints must use the same mode. Discarding command.")
            return
        mode = modes[0]
        
        try:
            if mode == 'mit_mode':
                positions = [float(getattr(joint, 'position', 0.0)) for joint in msg.joints]
                velocities = [float(getattr(joint, 'velocity', 0.0)) for joint in msg.joints]
                efforts = [float(getattr(joint, 'effort', 0.0)) for joint in msg.joints]
                
                kps = [float(param.get('mit_kp', 0.0)) for param in extra_params]
                kds = [float(param.get('mit_kd', 0.0)) for param in extra_params]
                
                mit_commands = [
                    MitMotorCommand(position=pos, speed=vel, torque=eff, kp=kp, kd=kd)
                    for pos, vel, eff, kp, kd in zip(positions, velocities, efforts, kps, kds)
                ]
                
                device.motor_command(CommandType.MIT, mit_commands)
                
            elif mode == 'position' or mode == 'position_mode':
                positions = [float(getattr(joint, 'position', 0.0)) for joint in msg.joints]
                device.motor_command(CommandType.POSITION, positions)
                
            elif mode == 'velocity' or mode == 'speed' or mode == 'speed_mode':
                velocities = [float(getattr(joint, 'velocity', 0.0)) for joint in msg.joints]
                device.motor_command(CommandType.SPEED, velocities)
                
            elif mode == 'torque' or mode == 'effort' or mode == 'torque_mode':
                torques = [float(getattr(joint, 'effort', 0.0)) for joint in msg.joints]
                device.motor_command(CommandType.TORQUE, torques)
                
            else:
                self.logw(f"Unknown {device_name} command mode: {mode}")
                return
            
        except Exception as e:
            self.loge(f"Error processing {device_name} joint command: {e}")

    def get_config_from_json(self, json_path: str) -> Optional[Dict]:
        try:
            with open(json_path, "r") as f:
                json_data = json.load(f)
                if json_data is not None:
                    if "joints" in json_data and isinstance(json_data["joints"], list):
                        self.logi(f"Load joint parameters from {json_path}.")
                        self.logi(f"Joint parameters: {json_data['joints']}.")
                        return json_data
                    else:
                        self.loge(f"Error: Have not found joints in {json_path}.")
                else:
                    self.loge(f"Error: JSON data is None in {json_path}.")
        except FileNotFoundError:
            self.loge(f"Error: File not found: {json_path}.")
        except json.JSONDecodeError:
            self.loge(f"Error: Failed to decode JSON from {json_path}.")
        except Exception as e:
            self.loge(f"Error: An unexpected error occurred while loading {json_path}: {e}.")
        
        return None
    
    def get_init_pose_config(self, json_path: str) -> Optional[Dict]:
        """Load init pose configuration including init_pose and step_limits.
        
        Returns:
            Dict with keys 'init_pose' and 'step_limits' if successful, None otherwise
        """
        try:
            with open(json_path, "r") as f:
                json_data = json.load(f)
                if json_data is not None:
                    if isinstance(json_data, dict):
                        result = {}
                        if 'init_pose' in json_data:
                            result['init_pose'] = json_data['init_pose']
                        if 'step_limits' in json_data:
                            result['step_limits'] = json_data['step_limits']
                        
                        if result:
                            self.logi(f"Load init pose config from {json_path}: {list(result.keys())}")
                            return result
                        else:
                            self.loge(f"Error: No 'init_pose' or 'step_limits' found in {json_path}.")
                    elif isinstance(json_data, list):
                        # Old format: just a list, treat as init_pose
                        self.logi(f"Load init pose from {json_path} (legacy format).")
                        return {'init_pose': json_data}
                    else:
                        self.loge(f"Error: Expected dict or list in {json_path}, got {type(json_data).__name__}.")
        except FileNotFoundError:
            self.loge(f"Error: File not found: {json_path}.")
        except json.JSONDecodeError:
            self.loge(f"Error: Failed to decode JSON from {json_path}.")
        except Exception as e:
            self.loge(f"Error: An unexpected error occurred while loading {json_path}: {e}.")
        return None
    