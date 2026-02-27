#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################
"""
HexDevice Python Library

A Python library for controlling HexDevice robots and devices.
Project link: https://github.com/hexfellow/hex_device_python
"""
# Core classes
from .device_base import DeviceBase
from .device_base_optional import OptionalDeviceBase
from .motor_base import (
    MotorBase, 
    MotorError, 
    MotorCommand, 
    CommandType, 
    MitMotorCommand,
    Timestamp
)

# Protobuf messages
from .generated import public_api_up_pb2, public_api_down_pb2, public_api_types_pb2

# Optional device implementations
from .hands import Hands
from .arm import Arm
from .chassis import Chassis

# Arm configuration system
from .arm_config import (
    ArmConfig,
    ArmConfigManager, 
    DofType,
    JointParam,
    JointParams,
    load_default_arm_config,
    get_arm_config,
    add_arm_config,
    arm_config_manager,
    set_arm_initial_positions,
    set_arm_initial_velocities,
    clear_arm_position_history,
    clear_arm_velocity_history,
    clear_arm_motion_history,
    get_arm_last_positions,
    get_arm_last_velocities
)

# Define what gets imported with "from hex_device import *"
__all__ = [
    # Core classes
    'DeviceBase',
    'OptionalDeviceBase',
    'MotorBase',
    'MotorError',
    'MotorCommand',
    'CommandType',
    'MitMotorCommand',
    'Timestamp',

    # Protobuf messages
    'public_api_up_pb2',
    'public_api_down_pb2',
    'public_api_types_pb2',

    # Optional device implementations
    'Hands',

    # Arm device
    'Arm',

    # Chassis device
    'Chassis',

    # Arm configuration system
    'ArmConfig',
    'ArmConfigManager',
    'DofType',
    'JointParam',
    'JointParams',
    'load_default_arm_config',
    'get_arm_config',
    'add_arm_config',
    'arm_config_manager',
    'set_arm_initial_positions',
    'set_arm_initial_velocities',
    'clear_arm_position_history',
    'clear_arm_velocity_history',
    'clear_arm_motion_history',
    'get_arm_last_positions',
    'get_arm_last_velocities',

]
