# hex_device_ros_wrapper

## Overview
This is a ROS2 package that provides ROS interface for hex robotic devices, including robotic arms and mobile chassis now.

## Prerequisites & Usage
### 1. Download the package
Create a workspace and download the package:
```bash
mkdir -p ~/hex_ws/src
cd ~/hex_ws/src
git clone --recursive https://github.com/hexfellow/hex_bridge.git
git clone --recursive https://github.com/hexfellow/hex_device_ros_wrapper.git
git clone --recursive https://github.com/hexfellow/hex_device_msgs.git
```
### 2. Install dependencies
```bash
cd ~/hex_ws/src/hex_device_ros_wrapper/
python3 -m pip install -r requirements.txt
```
### 3. Build
```bash
cd ~/hex_ws
colcon build
source install/setup.bash
```
### 4. Usage
Open a new terminal and run:
```bash
ros2 launch hex_bridge hex_bridge.launch.py url:={YOUR_IP}:8439
```
Open another terminal and run:  
If arm:
```bash
ros2 launch hex_device_ros_wrapper arm_bringup.launch.py
```
If chassis:
```bash
ros2 launch hex_device_ros_wrapper chassis_bringup.launch.py
```
For chassis, you can also run following commands to control chassis with keyboard:  
Open a new terminal and run:
```bash
ros2 run hex_device_ros_wrapper chassis_key_control
```

If lift:
```
ros2 launch hex_device_ros_wrapper lift_bringup.launch.py
```
For Lift, you can also run following commands to control Lift with simple pos command:
Open a new terminal and run:

```
ros2 topic pub /joint_cmd sensor_msgs/msg/JointState "{header: {}, name: ['joint1'], position: [0.3], velocity: [], effort: []}" --once
```

### 5.Parameter
**Clock Source Configuration**

You can use the `enable_ros_clock` parameter to decide whether to use the ROS clock as the message timestamp.
If the parameter is not assigned, ROS clock is used by default.
If set to `false`, the device controller internal clock source will be adopted for message timestamp.
```
ros2 launch hex_device_ros_wrapper lift_bringup.launch.py enable_ros_clock:=true
```

## Supported Devices

### 1. Arm
Provides interface for hex robotic arms with optional gripper support.  
You can remap topics as needed in the launch file.

#### Published Topics
| Topic             | Msg Type                   | Description                  |
| ----------------- | -------------------------- | ---------------------------- |
| `/ws_down`        | `std_msgs/UInt8MultiArray` | Protobuf messages to device  |
| `/joint_states`   | `sensor_msgs/JointState`   | Arm joint states             |
| `/gripper_states` | `sensor_msgs/JointState`   | Gripper states (if equipped) |

#### Subscribed Topics
| Topic          | Msg Type                                | Description                   |
| -------------- | --------------------------------------- | ----------------------------- |
| `/ws_up`       | `std_msgs/UInt8MultiArray`              | Protobuf messages from device |
| `/joints_cmd`  | `hex_device_msgs/XmsgArmJointParamList` | Arm joint control commands    |
| `/gripper_cmd` | `hex_device_msgs/XmsgArmJointParamList` | Gripper control commands      |

#### Parameters
| Parameter           | Type   | Default | Description                           |
| ------------------- | ------ | ------- | ------------------------------------- |
| `arm_series`        | int    | 0       | Arm series type (e.g., 16 for Archer) |
| `gripper_type`      | int    | 0       | Gripper type (0 = no gripper)         |
| `joint_config_path` | string | None    | Path to joint configuration file      |
| `init_pose_path`    | string | None    | Path to initial pose configuration    |

#### Message Type
`hex_device_msgs/XmsgArmJointParamList`:
```
XmsgArmJointParam[] joints
```
`XmsgArmJointParam`:
```
string mode
float64 position
float64 velocity
float64 effort
string extra_param
```
##### Example
There are some example to help you understand how to use this msg.  

Control a joint with the control mode set to position_mode, joint angle of 0.5 radians, velocity of 1.0 radians per second, torque of 1.0 Nm, and no additional parameters:
```bash
ros2 topic pub /joints_cmd hex_device_msgs/XmsgArmJointParamList "joints:
- {mode: 'position_mode', position: 0.5, velocity: 1.0, effort: 1.0, extra_param: ''}"
```
Open the brake when the joint is in position_mode:
```bash
ros2 topic pub /xtopic_arm/joints_cmd hex_device_msgs/XmsgArmJointParamList "joints:
- {mode: 'position_mode', position: 0.5, velocity: 1.0, effort: 1.0, extra_param: '{\"brake\": true}'}'"
```
Control a joint with the control mode set to mit_mode, joint angle of 0.5 radians, velocity of 1.0 radians per second, torque of 1.0 Newtons, and additional parameters: kp=1.0, kd=0.5:
```bash
ros2 topic pub /xtopic_arm/joints_cmd hex_device_msgs/XmsgArmJointParamList "joints:
- {mode:'mit_mode', position: 0.5, velocity: 1.0, effort: 1.0, extra_param: '{\"mit_kp\": 1.0, \"mit_kd\": 0.5}'}'"
```
For multiple joint control, just extend the list of commands.  
Check `pub_xmsg.py` file in hex_device_ros_wrapper package.

---

### 2. Chassis
Provides interface for hex mobile chassis with odometry support.  
You can remap topics as needed in the launch file.

#### Published Topics
| Topic           | Msg Type                   | Description                 |
| --------------- | -------------------------- | --------------------------- |
| `/ws_down`      | `std_msgs/UInt8MultiArray` | Protobuf messages to device |
| `/motor_states` | `sensor_msgs/JointState`   | Chassis motor states        |
| `/odom`         | `nav_msgs/Odometry`        | Chassis odometry            |

#### Subscribed Topics
| Topic        | Msg Type                   | Description                      |
| ------------ | -------------------------- | -------------------------------- |
| `/ws_up`     | `std_msgs/UInt8MultiArray` | Protobuf messages from device    |
| `/cmd_vel`   | `geometry_msgs/Twist`      | Velocity commands (simple mode)  |
| `/joint_cmd` | `sensor_msgs/JointState`   | Joint commands (advanced mode)   |
| `/clear_err` | `std_msgs/Bool`            | Clear error/parking stop request |

#### Parameters
| Parameter     | Type   | Default     | Description                                   |
| ------------- | ------ | ----------- | --------------------------------------------- |
| `frame_id`    | string | "base_link" | TF frame ID for odometry child frame          |
| `simple_mode` | bool   | true        | Simple mode (cmd_vel) vs advanced (joint_cmd) |

---


### 3. Lift
Provides interface for hex Lift support.Only the RtIotaVc2 is supported now.

#### Published Topics
| Topic           | Msg Type                   | Description                 |
| --------------- | -------------------------- | --------------------------- |
| `/ws_down`      | `std_msgs/UInt8MultiArray` | Protobuf messages to device |
| `/motor_states` | `sensor_msgs/JointState`   | Lift motor states (Only support position control) |


#### Subscribed Topics
| Topic        | Msg Type                   | Description                      |
| ------------ | -------------------------- | -------------------------------- |
| `/ws_up`     | `std_msgs/UInt8MultiArray` | Protobuf messages from device   |
| `/joint_cmd` | `sensor_msgs/JointState`   | Joint commands (Velocity represents the maximum movement speed. )|


---
