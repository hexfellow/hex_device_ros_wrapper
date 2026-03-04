#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import sys
import os
import time
script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)
from ros_interface import DataInterface
from hex_device_msgs.msg import XmsgArmJointParam, XmsgArmJointParamList

class XmsgInterface:
    def __init__(self, node_name: str):
        self.data_interface = DataInterface(node_name)
        self._joints_cmd_pub = self.data_interface.create_publisher("/joints_cmd", XmsgArmJointParamList)

    def pub_joints_cmd(self):
        msg = XmsgArmJointParamList(
            joints=[
                XmsgArmJointParam(mode="mit_mode", position=-0.3, velocity=0.0, effort=0.0, extra_param="{\"mit_kp\": 150.0, \"mit_kd\": 12.0}"),
                XmsgArmJointParam(mode="mit_mode", position=-1.48, velocity=0.0, effort=0.0, extra_param="{\"mit_kp\": 150.0, \"mit_kd\": 12.0}"),
                XmsgArmJointParam(mode="mit_mode", position=2.86, velocity=0.0, effort=0.0, extra_param="{\"mit_kp\": 150.0, \"mit_kd\": 12.0}"),
                XmsgArmJointParam(mode="mit_mode", position=0.0, velocity=0.0, effort=0.0, extra_param="{\"mit_kp\": 150.0, \"mit_kd\": 12.0}"),
                XmsgArmJointParam(mode="mit_mode", position=0.0, velocity=0.0, effort=0.0, extra_param="{\"mit_kp\": 39.0, \"mit_kd\": 0.8}"),
                XmsgArmJointParam(mode="mit_mode", position=0.0, velocity=0.0, effort=0.0, extra_param="{\"mit_kp\": 39.0, \"mit_kd\": 0.8}"),
                # XmsgArmJointParam(mode="position", position=-0.3, velocity=0.0, effort=0.0, extra_param=""),
                # XmsgArmJointParam(mode="position", position=-1.48, velocity=0.0, effort=0.0, extra_param=""),
                # XmsgArmJointParam(mode="position", position=2.86, velocity=0.0, effort=0.0, extra_param=""),
                # XmsgArmJointParam(mode="position", position=0.0, velocity=0.0, effort=0.0, extra_param=""),
                # XmsgArmJointParam(mode="position", position=0.0, velocity=0.0, effort=0.0, extra_param=""),
                # XmsgArmJointParam(mode="position", position=0.0, velocity=0.0, effort=0.0, extra_param=""),
            ]
        )
        self.data_interface.publish(self._joints_cmd_pub, msg)

def main():
    xmsg_interface = XmsgInterface("pub_xmsg")
    control_hz = 100.0
    try:
        period_time = 1.0 / control_hz
        next_time = time.perf_counter()
        while True:
            next_time += period_time
            xmsg_interface.pub_joints_cmd()
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time) 
    except KeyboardInterrupt:
        xmsg_interface.data_interface.shutdown() 
    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()