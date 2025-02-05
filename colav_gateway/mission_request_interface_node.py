import socket
import asyncio
import rclpy
from rclpy.node import Node
from mission_request.mission_request import MissionRequestNode
import argparse
import json
import os
import rclpy.logging
from enum import Enum
import sys

# Protobuf imports
from missionRequest_pb2 import MissionRequest as ProtobufMissionRequest

# ROS interface imports
# Mission Request
from colav_interfaces.msg import MissionRequest as RosMissionRequest # This is goal msg for service
from colav_interfaces.action import MissionExecutor
from std_msgs.msg import ByteMultiArray # this if feedback message

import numpy as np
from geometry_msgs.msg import Point32
from rclpy.action import ActionClient
from rclpy.impl.rcutils_logger import RcutilsLogger

from typing import Any, Tuple

from example_interfaces.action import Fibonacci

ROS_NAMESPACE = "colav_gateway"
NODE_NAME = "mission_request_listener"
ACTION_NAME = "execute_colav_mission"

logger = rclpy.logging.get_logger(f"{ROS_NAMESPACE}/{NODE_NAME}")

class ColavGatewayMissionRequestListener(Node):
    """ColavGateway acts as the gateway to the COLAV application. It listens to UDP sockets
    and translates received Protobuf messages into ROS messages for COLAV topics."""

    def __init__(self, config: dict):
        """Initializes the ColavGatewayNode."""
        super().__init__(namespace=ROS_NAMESPACE, node_name=NODE_NAME)
        self.get_logger().info(f"{ROS_NAMESPACE}/{NODE_NAME}: Node initializing...")
        
        mission_request_config = config.get("endpoint_config", {}).get("mission_request", {})
        self._mission_request_address = (
            mission_request_config.get("host", "localhost"),
            mission_request_config.get("port", 0),
        )
        
        # self.ctrl_execution_client = ActionClient(
        #     self, 
        #     MissionExecutor,  # action type
        #     "/colav_gateway/execute_mission"
        # )

        self.ctrl_execution_client = ActionClient(
            self, 
            Fibonacci,  # action type
            "/fibonacci"
        )
        
        if not self.ctrl_execution_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available.")
            raise RuntimeError("Action server not available.")

    async def listen_for_mission_request(self):
        """Starts the UDP listener for mission requests."""
        self.get_logger().info("Initiating COLAV Mission Request Listener.")
        loop = asyncio.get_running_loop()
        transport, protocol = await loop.create_datagram_endpoint(
            lambda: self.MissionRequestUDPProtocol(self.ctrl_execution_client, self.get_logger()),
            local_addr=self._mission_request_address,
        )
        return transport, protocol


    class MissionRequestUDPProtocol(asyncio.DatagramProtocol):
        """Handles incoming UDP mission requests and forwards them to the COLAV system."""

        def __init__(self, ctrl_execution_client: ActionClient, logger: RcutilsLogger):
            """Initializes the MissionRequestUDPProtocol."""
            self._ctrl_execution_action_client = ctrl_execution_client # ActionClient for mission execution
            self._logger = logger

        def connection_made(self, transport: asyncio.DatagramTransport):
            self.transport = transport

        def datagram_received(self, data: bytes, addr: Tuple[Any, int]):
            self._logger.info(f"Received data from {addr}")
            ros_mission_request = self._protobuf_mission_request_to_ros_mission_request(data)
            asyncio.create_task(self._process_datagram(data, addr))

        async def _process_datagram(self, data: bytes, addr: Tuple[Any, int]):
            """Process the datagram asynchronously."""
            try:
                future = self._send_goal(10)
                rclpy.spin_until_future_complete(
                    self._ctrl_execution_action_client,
                    future
                )
                # self._ctrl_execution_action_client.wait_for_server()
                # send_goal_future = self._ctrl_execution_action_client.send_goal_async(
                #     goal_msg,
                #     # feedback_callback=self.ctrl_feedback_callback
                # )
                # self._logger.info(goal_msg)
                # send_goal_future.add_done_callback(self._ctrl_goal_response_callback)
                    
                # Adding a timeout to the future
                # loop = asyncio.get_event_loop()
                # loop.call_later(10, self._check_goal_future, send_goal_future)
            except Exception as e:
                self._logger.error(f"Error processing mission request: {e}")

        def _send_goal(self, order):
            goal_msg = Fibonacci.Goal()
            goal_msg.order = order
            self._ctrl_execution_action_client.wait_for_server()
            return self._ctrl_execution_action_client.send_goal_async(
                goal_msg,
                # feedback_callback=self.ctrl_feedback_callback
            )

        def _check_goal_future(self, future):
            if not future.done():
                self._logger.error("send_goal_async did not complete within the timeout period.")
                future.cancel()

        def _ctrl_goal_response_callback(self, future):
            """Handles the response from the mission execution action server."""
            pass

        def _protobuf_mission_request_to_ros_mission_request(self, protobuf_mission_request: ProtobufMissionRequest) -> RosMissionRequest:
            """Converts a protobuf mission request to a ROS MissionRequest message."""
            mission_request_msg = ProtobufMissionRequest()
            mission_request_msg.ParseFromString(protobuf_mission_request)
            return self._convert_to_ros_mission(mission_request_msg)
            
        def _convert_to_ros_mission(self, mission_request_msg: 'missionRequest_pb2.MissionRequest') -> RosMissionRequest:
            """Converts a protobuf mission request to a ROS MissionRequest message."""
            ros_mission_msg = RosMissionRequest()
            ros_mission_msg.mission_tag = mission_request_msg.tag
            ros_mission_msg.mission_sent_timestamp = mission_request_msg.mission_start_timestamp
            ros_mission_msg.vessel.tag = mission_request_msg.vessel.tag
            ros_mission_msg.vessel.type = ProtobufMissionRequest.Vessel.VesselType.Name(
                mission_request_msg.vessel.type
            )
            ros_mission_msg.vessel.dynamic_constraints.max_acceleration = (
                mission_request_msg.vessel.vessel_constraints.max_acceleration
            )
            ros_mission_msg.vessel.dynamic_constraints.max_deceleration = (
                mission_request_msg.vessel.vessel_constraints.max_deceleration
            )
            ros_mission_msg.vessel.dynamic_constraints.max_velocity = (
                mission_request_msg.vessel.vessel_constraints.max_velocity
            )
            ros_mission_msg.vessel.dynamic_constraints.min_velocity = (
                mission_request_msg.vessel.vessel_constraints.min_velocity
            )
            ros_mission_msg.vessel.dynamic_constraints.max_yaw_rate = (
                mission_request_msg.vessel.vessel_constraints.max_yaw_rate
            )
            ros_mission_msg.vessel.geometry.acceptance_radius = (
                mission_request_msg.vessel.vessel_geometry.safety_threshold
            )
            ros_mission_msg.vessel.geometry.polyshape.points = [
                Point32(x=point.x, y=point.y, z=point.z)
                for point in mission_request_msg.vessel.vessel_geometry.polyshape_points
            ]
            ros_mission_msg.mission_init_position = Point32(
                x=mission_request_msg.mission_init_position.x,
                y=mission_request_msg.mission_init_position.y,
                z=mission_request_msg.mission_init_position.z,
            )
            ros_mission_msg.mission_goal_position = Point32(
                x=mission_request_msg.mission_goal_position.x,
                y=mission_request_msg.mission_goal_position.y,
                z=mission_request_msg.mission_goal_position.z,
            )
            return ros_mission_msg

        def ctrl_feedback_callback(self, controller_feedback: ByteMultiArray):
            self._logger.info("Received controller feedback.")
            self._logger.info(controller_feedback)

        def connection_lost(self, exc):
            self._logger.info("UDP connection lost.")


def main():
    parser = argparse.ArgumentParser(description="COLAV Gateway Configuration")
    parser.add_argument("-c", "--config", type=str, required=True, help="Path to config JSON file.")
    args = parser.parse_args()

    try:
        with open(args.config, "r") as config_file:
            colav_gateway_config = json.load(config_file)
            logger.info("Config file successfully loaded.")
    except Exception as e:
        logger.error(f"Failed to load config: {e}")
        sys.exit(1)

    rclpy.init(args=None)
    gateway_node = ColavGatewayMissionRequestListener(config=colav_gateway_config)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    executor = rclpy.executors.MultiThreadedExecutor()
    loop.run_in_executor(None, lambda: rclpy.spin(gateway_node))
    loop.run_until_complete(gateway_node.listen_for_mission_request())

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        logger.info("Shutting down COLAV Gateway.")
    finally:
        gateway_node.destroy_node()
        rclpy.shutdown()
        loop.close()


if __name__ == "__main__":
    logger.info("Starting COLAV Gateway.")
    main()