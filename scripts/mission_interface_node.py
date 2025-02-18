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
from action_msgs.msg import GoalStatus

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
            node_name:str = "mission_interface_node", 
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
        self.get_logger().info(f"Checking /colav_gateway/execute_mission action srv is active before listening for mission request...")
        # need to create an action client.
        execute_mission_cli = ActionClient(
            self,
            MissionExecutor,
            '/colav_gateway/execute_mission'
        )
        if not execute_mission_cli.wait_for_server(timeout_sec=action_srv_timeout):
            raise Exception(f"/execute_mision action srv not online therefore stopping node.")
        
        self.get_logger().info(f"/exeucte_mission action active, Listening for mission requests!")
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
            feedback_callback=self.ctrl_feedback_callback
        )
        send_goal_future.add_done_callback(self._ctrl_goal_response_callback)

    def ctrl_feedback_callback(self, feedback:MissionExecutor.Feedback, timeout:float = 5.0):

        try: 
            self.get_logger().info(f'Feedback received: {feedback}')
            controller_feedback = bytes(feedback.serialised_protobuf_controller_feedback.data)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(controller_feedback, self._controller_feedback_address)
            self.get_logger(f'Controller feedback sent to {self._controller_feedback_address}')
        except Exception as e: 
            self.get_logger().error(f'{e}')
        finally:
            sock.close()
        # Receiving feedback in byte format in feedback.serialized msg now need to send this to controller feedback socket until mission is completed status is returned!!!

    def _ctrl_goal_response_callback(self, future):

        goal_handle = future.result()
        mission_response_msg = ""
        if goal_handle.accepted:
            pass
        else: 
            pass
        # if result:
        #     goal_status = result.status
        #     if goal_status == GoalStatus.STATUS_UNKNOWN:
        #         self.get_logger().info('The goal status is unknown.')
        #     elif goal_status == GoalStatus.STATUS_ACCEPTED:
        #         self.get_logger().info('Goal has been ACCEPTED!')
        #     elif goal_status == GoalStatus.STATUS_EXECUTING:
        #         self.get_logger().info('Goal is EXECUTING!')
        #     elif goal_status == GoalStatus.STATUS_CANCELING:
        #         self.get_logger().info('Goal is being CANCELLED.')
        #     elif goal_status == GoalStatus.STATUS_SUCCEEDED:
        #         self.get_logger().info('Goal has SUCCEEDED!')
        #     elif goal_status == GoalStatus.STATUS_CANCELED:
        #         self.get_logger().info('Goal was CANCELED!')
        #     elif goal_status == GoalStatus.STATUS_ABORTED:
        #         self.get_logger().info('Goal was ABORTED!')
        #     elif goal_status == GoalStatus.STATUS_REJECTED:
        #         self.get_logger().info('Goal was REJECTED!')
        #     elif goal_status == GoalStatus.STATUS_PREEMPTED:
        #         self.get_logger().info('Goal was PREEMPTED by a new goal!')
        # else:
        #     self.get_logger().info('No result returned for the mission!')
