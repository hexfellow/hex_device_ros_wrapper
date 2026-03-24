from enum import Enum
import time


from .ros_interface import DataInterface
from .hex_device_py import linear_lift, zeta_lift , CommandType, public_api_up_pb2, public_api_down_pb2, public_api_types_pb2, Timestamp
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState


import traceback


class LiftType(Enum):
    LiftNone = 0
    Linear = 1
    Rotate = 2

class ClassLinearLiftApi:
    
    def __init__(self):
        self.ros_interface = DataInterface("xnode_lift")
        
        self._status = False
        
        self.robot_type = None
        self.Lift = None
        self.lift_type = None
        
        self.last_update_time = Timestamp.from_ns(time.perf_counter_ns())
        
        
        # ============create_subscript_publisher=============
        self.ws_up_sub = self.ros_interface.create_subscription(
            'ws_up', UInt8MultiArray, self._ws_up_callback, 10)
        
        self.ws_down_pub = self.ros_interface.create_publisher('ws_down', UInt8MultiArray, 10)
    
    
    
    async def _pub_ws_down(self, data):
        """Unified ws_down publishing function, shared by all devices"""
        try:
            msg = UInt8MultiArray()
            msg.data = data.SerializeToString()
            self.ros_interface.publish(self.ws_down_pub, msg)
        except Exception:
            pass
    
    
    
    def _ws_up_callback(self, msg):
        api_up = public_api_up_pb2.APIUp()
        api_up.ParseFromString(bytes(msg.data))
        
        
        # ========== check lift ========
        if self._status == False and api_up.HasField("linear_lift_status"):
            
            self._status = True
            self.robot_type = api_up.robot_type
            self.lift_type = LiftType.Linear
            
            self.Lift = linear_lift.LinearLift(
                motor_count=1,
                robot_type=api_up.robot_type,
                
                send_message_callback = self._pub_ws_down
            )
            
        elif self._status == False and api_up.HasField("rotate_lift_status"):
            
            self._status = True
            self.robot_type = api_up.robot_type
            self.lift_type = LiftType.Rotate
            motor_counts = len(motor_status)
            
            motor_status = api_up.rotate_lift_status.motor_status
            
            self.Lift = zeta_lift.ZetaLift(
                motor_count=motor_counts,
                robot_type=api_up.robot_type,
                
                send_message_callback = self._pub_ws_down
            )
            
        elif self._status == True:
            self.last_update_time = Timestamp.from_ns(time.perf_counter_ns())
            
            self.Lift._update(api_up,self.last_update_time)
            
        
        print(f"\n msg=========================\n{api_up}")
    
    # ====== callback=====
    def _joint_cmd_callback(self):
        pass
    
    
    # def __linear_send_callback(self):
    #     pass
    
    # def __zeta_send_callback(self):
    #     pass

def main():
    
    api = ClassLinearLiftApi()

    try:
        while api.ros_interface.ok():
            pass
    except KeyboardInterrupt:
        api.ros_interface.shutdown()
    
    
    
if __name__ == '__main__':
    main()
