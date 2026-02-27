#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import threading
import rclpy
import rclpy.node
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from .interface_base import InterfaceBase


class DataInterface(InterfaceBase):
    """
    ROS2 interface implementation
    """

    def __init__(self, name: str):
        super(DataInterface, self).__init__(name=name)

        # Initialize ROS2 node
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()

        # Initialize rate control
        self._rate = self.__node.create_rate(100)

        # Start spin thread
        self.__spin_thread = threading.Thread(target=self.spin)
        self.__spin_thread.daemon = True
        self.__spin_thread.start()

    # ========== Topic management (generic) ==========

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
        try:
            return self.__node.create_publisher(msg_type, topic_name, queue_size)
        except Exception as e:
            self.loge(f"Failed to create publisher for {topic_name}: {e}")
            return None

    def create_subscription(self, topic_name: str, msg_type: type,
                           callback, queue_size: int = 10):
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
        try:
            return self.__node.create_subscription(
                msg_type, topic_name, callback, queue_size)
        except Exception as e:
            self.loge(f"Failed to create subscription for {topic_name}: {e}")
            return None

    def publish(self, publisher, msg):
        """
        Publish message

        Args:
            publisher: Publisher object (returned by create_publisher)
            msg: Message object
        """
        try:
            publisher.publish(msg)
        except Exception:
            pass

    # ========== Timer ==========

    def create_timer(self, interval_sec: float, callback):
        """
        Create timer

        Args:
            interval_sec: Timer interval (seconds)
            callback: Callback function (no parameters, defined externally)

        Returns:
            Timer object
        """
        try:
            return self.__node.create_timer(interval_sec, callback)
        except Exception as e:
            self.loge(f"Failed to create timer: {e}")
            return None

    # ========== Parameter server ==========

    def get_parameter(self, name: str):
        """
        Get parameter

        Args:
            name: Parameter name
        """
        try:
            return self.__node.get_parameter(name).value
        except Exception as e:
            self.loge(f"Failed to get parameter: {e}")

    def set_parameter(self, name: str, value):
        """
        Set parameter

        Args:
            name: Parameter name
            value: Parameter value
        """
        try:
            self.__node.declare_parameter(name, value)
        except Exception as e:
            self.loge(f"Failed to set parameter {name}: {e}")

    # ========== Rate control ==========

    def set_rate(self, hz: float):
        """
        Set rate controller frequency

        Args:
            hz: Frequency (Hz)
        """
        try:
            self._rate = self.__node.create_rate(hz)
        except Exception as e:
            self.loge(f"Failed to set rate: {e}")

    def sleep(self):
        """Sleep according to rate"""
        self._rate.sleep()

    # ========== Lifecycle ==========

    def ok(self) -> bool:
        """Check if ROS is running normally"""
        return rclpy.ok()

    def shutdown(self):
        """Shutdown ROS node"""
        try:
            self.__node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

    # ========== Logging ==========

    def logd(self, msg: str, *args, **kwargs):
        """Output debug log"""
        try:
            self.__logger.debug(msg, *args, **kwargs)
        except Exception:
            pass

    def logi(self, msg: str, *args, **kwargs):
        """Output info log"""
        try:
            self.__logger.info(msg, *args, **kwargs)
        except Exception:
            pass

    def logw(self, msg: str, *args, **kwargs):
        """Output warn log"""
        try:
            self.__logger.warning(msg, *args, **kwargs)
        except Exception:
            pass

    def loge(self, msg: str, *args, **kwargs):
        """Output error log"""
        try:
            self.__logger.error(msg, *args, **kwargs)
        except Exception:
            pass

    def logf(self, msg: str, *args, **kwargs):
        """Output fatal log"""
        try:
            self.__logger.fatal(msg, *args, **kwargs)
        except Exception:
            pass

    # ========== Tools ==========
   
    def get_timestamp(self):
        return self.__node.get_clock().now().to_msg()
    
    def get_timestamp_from_s_ns(self, s: int, ns: int):
        return Time(seconds=s, nanoseconds=ns).to_msg()

    # ========== Internal methods ==========

    def spin(self):
        """Spin ROS2 node in separate thread"""
        try:
            rclpy.spin(self.__node)
        except Exception:
            pass

    def spin_once(self):
        """Spin ROS2 node once"""
        try:
            rclpy.spin_once(self.__node)
        except Exception:
            pass