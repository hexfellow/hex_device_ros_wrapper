import time
import threading
import asyncio
import signal

import os
import sys
script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)

from ros_interface import DataInterface
from hex_device_py import linear_lift, zeta_lift , CommandType, public_api_up_pb2, public_api_down_pb2, public_api_types_pb2, Timestamp
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState

import traceback

class ClassLinearLiftApi:
    
    def __init__(self):
        
        self.ros_interface = DataInterface("xnode_lift")
        
        self._status = False
        self._stop_event = None
        
        self.robot_type = None
        self.Lift = None
        
        self.last_update_time = Timestamp.from_ns(time.perf_counter_ns())
        self._asnyc_loop = None
        
        # Get parameters
        self.ros_interface.set_parameter('enable_ros_clock', True)
        self.enable_ros_clock = self.ros_interface.get_parameter('enable_ros_clock')
        
        # ============ Subscribers & Publishers ============
        self.ws_up_sub = self.ros_interface.create_subscription(
            'ws_up', UInt8MultiArray, self._ws_up_callback, 10)
        
        self.ws_down_pub = self.ros_interface.create_publisher('ws_down', UInt8MultiArray, 10)
        self.joint_cmd_sub = None
        self.motor_states_pub = None
        
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
            if self.Lift is not None:
                self.last_update_time = Timestamp.from_ns(time.perf_counter_ns())
                self.Lift._update(api_up,self.last_update_time)
                self._publish_motor_states()
        else:
            self._dev_init(api_up)
    
    # ====== callback =====
    def _joint_cmd_callback(self, msg):
        try :
            msg.position = list(msg.position)
                
            if isinstance(self.Lift, linear_lift.LinearLift):
                if len(msg.position) == 1:
                    self.Lift.motor_command(CommandType.POSITION,msg.position[0])
                else:
                    self.ros_interface.logw("joint_callback waring: Position count != 1")
                        
        except Exception:
            print(f"Error in linear_joint_cmd_callback:  \n {traceback.format_exc()}")

    def _publish_motor_states(self):
        try:
            position_status =None
            msg = None
            
            if self.Lift is not None :
                if isinstance(self.Lift,linear_lift.LinearLift):
                    _timestamp = self._get_clock_timestamp()
                    position_status = self.Lift.get_motor_positions()
                    velocity_status = self.Lift.get_move_speed() / self.Lift._pulse_per_rotation
                    msg = JointState()
                    msg.header.stamp = _timestamp
                    msg.name = [f"joint1"]
                    msg.position = [position_status]
                    msg.velocity = [velocity_status]
                    msg.effort = [0.0]
            
            if position_status is None:
                self.ros_interface.logi("position_status is None")
                return
            
            if msg is not None:
                self.ros_interface.publish(self.motor_states_pub, msg)
        
        except Exception:
            self.ros_interface.loge(f"Error in _publish_motor_states:  \n {traceback.format_exc()}")

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
            
            self.ros_interface.logw("Zeta Lift is not supported yet.")
            self.Lift = None
            self._stop_event.set()
            
            # motor_status = msg.rotate_lift_status.motor_status
            # motor_counts = len(motor_status)
            # self.Lift = zeta_lift.ZetaLift(
            #     motor_count=motor_counts,
            #     robot_type=msg.robot_type,
                
            #     send_message_callback = self._pub_ws_down
            # )

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
            
            self.motor_states_pub = self.ros_interface.create_publisher(
                '/xtopic_lift/motor_states', 
                JointState, 
                10
            )
            
            self.ros_interface.logi(f"Get ssid: {msg.session_id}")

    def set_stop_event(self,event):
        self._stop_event = event
        
    # ========== tool ==========
        
    def _get_clock_timestamp(self):
        _timestamp = None
        
        device = self.Lift

        if self.enable_ros_clock == True:
            _timestamp = self.ros_interface.get_timestamp()
            
        elif self.enable_ros_clock == False:
            _timestamp = self.ros_interface.get_timestamp_from_s_ns(device._last_update_time.s, device._last_update_time.ns)
        
        return _timestamp

# ====== task ========
def _run_async_thread(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()
    
def cleanup(ros_node, loop, threads:list):
    ros_node.shutdown()
    def stop_loop():
        tasks = asyncio.all_tasks(loop)

        for t in tasks:
            t.cancel()

        # 给任务一点时间处理 CancelledError
        loop.call_later(0.1, loop.stop)
    loop.call_soon_threadsafe(stop_loop)
    
    for thread_task in threads:
        thread_task.join(timeout=2.0)

# ========= Signal Handler =========
def signal_handler(signum, frame,event):
    if event.is_set():
        return
    print(f"\n[Signal] Interrupt received ({signum}), shutting down...")
    event.set()

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
    
    signal.signal(signal.SIGINT, lambda s, f: signal_handler(s, f, shutdown_event))
    signal.signal(signal.SIGTERM, lambda s, f: signal_handler(s, f, shutdown_event))
    
    api = ClassLinearLiftApi()
    api._asnyc_loop = _asnyc_loop
    api.set_stop_event(shutdown_event)
    
    try:
        _asnyc_loop_thread.start()
        shutdown_event.wait()
        
    except KeyboardInterrupt:
        pass
    
    finally:
        cleanup(api.ros_interface,_asnyc_loop,[_asnyc_loop_thread])
        api.ros_interface.logi("finally clean")
        

if __name__ == '__main__':
    main()
