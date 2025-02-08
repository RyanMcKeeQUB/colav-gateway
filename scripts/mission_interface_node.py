import socket
import asyncio
import rclpy
from rclpy.node import Node
import argparse
import json
import os
import rclpy.logging
from enum import Enum
import sys

# Protobuf imports
from proto_gen.missionRequest_pb2 import MissionRequest as ProtobufMissionRequest

# ROS interface imports
# Mission Request
from colav_interfaces.msg import MissionRequest as RosMissionRequest # This is goal msg for service
from colav_interfaces.action import MissionExecutor
from std_msgs.msg import ByteMultiArray # this if feedback message

import numpy as np
from geometry_msgs.msg import Point32
from rclpy.action import ActionClient
from rclpy.impl.rcutils_logger import RcutilsLogger
sys.path.append(os.path.abspath('/home/3507145@eeecs.qub.ac.uk/Documents/ColavProject/colav_ws/src/colav_server/colav_gateway'))
from typing import Any, Tuple
from utils.udp_socket_utils import setup_udp_socket
from example_interfaces.action import Fibonacci

from utils.proto_ros_converter_utils import ProtoToROSUtils

ROS_NAMESPACE = "colav_gateway"
NODE_NAME = "mission_request_listener"
ACTION_NAME = "execute_colav_mission"

logger = rclpy.logging.get_logger(f"{ROS_NAMESPACE}/{NODE_NAME}")

class MissionInterfaceNode(Node):
    """ColavGateway acts as the gateway to the COLAV application. It listens to UDP sockets
    and translates received Protobuf messages into ROS messages for COLAV topics."""

    def __init__(
            self, 
            namespace:str = "colav_gateway", 
            node_name:str = "mission_interface", 
            is_thread: bool = False,
            threading_events: dict = None
    ):
        """Initializes the ColavGatewayNode."""
        super().__init__(namespace=namespace, node_name=node_name)

        self.declare_parameter('mission_request_host', '0.0.0.0')
        self.declare_parameter('mission_request_port', 7000)
        self.declare_parameter('mission_response_host', '0.0.0.0')
        self.declare_parameter('mission_response_port', 7001)
        self.declare_parameter('controller_feedback_host', '0.0.0.0')
        self.declare_parameter('controller_feedback_port', 7300)
        
        # self._mission_request_address = mission_request_address
        # self._mission_response_address = mission_response_address
        self._mission_request_address = (
            self.get_parameter('mission_request_host').get_parameter_value().string_value,
            self.get_parameter('mission_request_port').get_parameter_value().integer_value
        )
        self._mission_response_address = (
            self.get_parameter('mission_response_host').get_parameter_value().string_value,
            self.get_parameter('mission_response_port').get_parameter_value().integer_value
        )
        self._controller_feedback_address = (
            self.get_parameter('controller_feedback_host').get_parameter_value().string_value, 
            self.get_parameter('controller_feedback_port').get_parameter_value().integer_value
        )

        if is_thread:
            self._threading_events = threading_events
        self.get_logger().info(f"{namespace}/{node_name}: node initialised.")
        self.listen_for_mission_request()

    def listen_for_mission_request(self, action_srv_timeout: float = 5.0):
        """Starts the UDP listener for mission requests."""
        self.get_logger().info(f"Listening for mission requests")

        # need to create an action client.
        execute_mission_cli = ActionClient(
            self,
            MissionExecutor,
            '/colav_gateway/execute_mission'
        )
        if not execute_mission_cli.wait_for_server(timeout_sec=action_srv_timeout):
            raise Exception("/execute_mision action srv not online therefore pausing node.")
                    
        sock = setup_udp_socket(address=self._mission_request_address, timeout=1.0)

        while True:
            try: 
                data, client_address = sock.recvfrom(1024)
                self.get_logger().info(f"Mission Request protobuf received from {client_address}")
                mission_request = ProtoToROSUtils.parse_mission_request(data)
                if mission_request: 
                    self.execute_mission(execute_mission_cli, mission_request=mission_request)
            except TimeoutError:
                continue
            except KeyboardInterrupt:
                self.get_logger().info("Shutting down mission listener!")
                break
            except Exception as e: 
                self.get_logger().warning(e)
                continue
            
        sock.close()

    def execute_mission(self, execute_mission_cli: ActionClient, mission_request: RosMissionRequest): 
        req = MissionExecutor.Goal()
        req.mission_request = mission_request
        send_goal_future = execute_mission_cli.send_goal_async(
            goal=req,
            feedback_callback=lambda: self.ctrl_feedback_callback()
        )
        send_goal_future.add_done_callback(self._ctrl_goal_response_callback)

    def ctrl_feedback_callback(self, timeout:float = 5.0):
        self.get_logger().info('Feedback received!')

    def _ctrl_goal_response_callback(self):
        self.get_logger().info('mission completed!')