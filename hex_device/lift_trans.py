from enum import Enum
import time
import threading
import asyncio
import signal

from .ros_interface import DataInterface
from .hex_device_py import linear_lift, zeta_lift , CommandType, public_api_up_pb2, public_api_down_pb2, public_api_types_pb2, Timestamp
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
from hex_device_msgs.msg import XmsgArmJointParamList

import traceback


class ClassLinearLiftApi:
    
    def __init__(self):
        
        # ================== Get parameters =======================
        # 限幅检查
        
        self._Lift_Max_position=0.6  # Unit: m
        self._Lift_Min_position=0.1
        
        self.ros_interface = DataInterface("xnode_lift")
        
        self._status = False
        self._status_lock = threading.Lock()
        
        
        
        self.robot_type = None
        self.Lift = None
        
        self.last_update_time = Timestamp.from_ns(time.perf_counter_ns())
        self._asnyc_loop = None
        
         # ============ Subscribers & Publishers ============
        self.ws_up_sub = self.ros_interface.create_subscription(
            'ws_up', UInt8MultiArray, self._ws_up_callback, 10)
        
        self.ws_down_pub = self.ros_interface.create_publisher('ws_down', UInt8MultiArray, 10)
        
        
        self.joint_cmd_sub = None
        
    async def _pub_ws_down(self, data):
        """Unified ws_down publishing function, shared by all devices"""
        try:
            msg = UInt8MultiArray()
            msg.data = data.SerializeToString()
            self.ros_interface.publish(self.ws_down_pub, msg)
        except Exception:
            print(f"Error in linear_joint_cmd_callback:  \n {traceback.format_exc()}")
    
    def _ws_up_callback(self, msg):
        api_up = public_api_up_pb2.APIUp()
        api_up.ParseFromString(bytes(msg.data))
        
        if self._status:
            self.last_update_time = Timestamp.from_ns(time.perf_counter_ns())
            self.Lift._update(api_up,self.last_update_time)
        else:
            self._dev_init(api_up)
    
    # ====== callback =====
    def _joint_cmd_callback(self, msg):
        try :
            msg.position = list(msg.position)
            print(f"joint msg\n:{msg}")
                
            if isinstance(self.Lift, linear_lift.LinearLift):
                if len(msg.position) == 1:
                    self.Lift.motor_command(CommandType.POSITION,msg.position[0])
                else:
                    self.ros_interface.logw("joint_callback waring: Position count != 1")
            
            if isinstance(self.Lift, zeta_lift.ZetaLift):
                if len(msg.position) == 3:
                    self.Lift.motor_command(CommandType.POSITION,msg.position)
                else:
                    self.ros_interface.logw("joint_callback waring: Position count != 3")
                        
        except Exception:
            print(f"Error in linear_joint_cmd_callback:  \n {traceback.format_exc()}")

    # ====== device init ======
    def _dev_init(self,msg):
        robot_type = msg.robot_type
        if robot_type not in zeta_lift.ZetaLift.SUPPORTED_ROBOT_TYPES and robot_type not in linear_lift.LinearLift.SUPPORTED_ROBOT_TYPES:
            self.ros_interface.logw(f"Robot type mismatch!")
            return
        
        # ========== check lift ========
        if  msg.HasField("linear_lift_status"):
            
            self.Lift = linear_lift.LinearLift(
                motor_count=1,
                robot_type=msg.robot_type,
                send_message_callback = self._pub_ws_down
            )
            
        if msg.HasField("rotate_lift_status"):
        
            motor_status = msg.rotate_lift_status.motor_status
            motor_counts = len(motor_status)
            
            self.Lift = zeta_lift.ZetaLift(
                motor_count=motor_counts,
                robot_type=msg.robot_type,
                
                send_message_callback = self._pub_ws_down
            )

        # ======== lift init ============
        if self.Lift is not None:
            self._status = True
            
            self.Lift.start()
            self.Lift.calibrate()
            self.ros_interface.logi("Hex_device Init")
            # ========= create asyncio loop ===========
            if self._asnyc_loop is not None:
                try:
                    asyncio.run_coroutine_threadsafe(self.Lift._periodic(), self._asnyc_loop)
                except:
                    print(f"Error in _asnyc_loop:\n {traceback.format_exc()}")
            else:
                print(f"self._asnyc_loop{type(self._asnyc_loop)}")
            # ========= create publisher ============
            self.joint_cmd_sub = self.ros_interface.create_subscription(
                '/xtopic_lift/joint_cmd',
                JointState,
                self._joint_cmd_callback, 
                10
            )

# ====== task ========
def _run_async_thread(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()
    


# ========= Signal Handler =========
def signal_handler(event,signum, frame):
    if event.is_set():
        return
    print(f"\n[Signal] Interrupt received ({signum}), shutting down...")
    event.set()
    raise KeyboardInterrupt


def main():
    # ========= asnyc thread =========
    _asnyc_loop = asyncio.new_event_loop()
    _asnyc_loop_thread = threading.Thread(
            target=_run_async_thread, 
            args=(_asnyc_loop,), 
            daemon=True
        )
    
    # ========= Setup Events =========
    shutdown_event = threading.Event()
    
    signal.signal(signal.SIGINT, lambda s, f: signal_handler(shutdown_event, s, f))
    signal.signal(signal.SIGTERM, lambda s, f: signal_handler(shutdown_event, s, f))
    
    api = ClassLinearLiftApi()
    api._asnyc_loop = _asnyc_loop
    
    try:
        _asnyc_loop_thread.start()
        
        shutdown_event.wait()
        
    except KeyboardInterrupt:
        pass
    finally:
        print("main finally")
        api.ros_interface.shutdown()
        _asnyc_loop.call_soon_threadsafe(_asnyc_loop.stop)
        _asnyc_loop_thread.join(timeout=1.0)
        

if __name__ == '__main__':
    main()
