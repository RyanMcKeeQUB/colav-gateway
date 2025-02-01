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

import missionRequest_pb2
import controllerFeedback_pb2

from colav_interfaces.msg import MissionRequest
from colav_interfaces.action import MissionExecutor
import numpy as np
from geometry_msgs.msg import Point32
from rclpy.action import ActionClient

logger = rclpy.logging.get_logger("colav_gateway_logger")
workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class ColavGatewayNode(Node):
    """ColavGateWay acts as the gateway to the COLAV application. Listening to respective
    UDP sockets depending on the internal state of the colav application and translating
    the UDP Protobuf msgs to internal ros messages which will be multicast published to
    COLAV topics.

    Args:
        Node (_type_): ROS2 Node
    """

    # TODO Need to make this a a request to action server for controller
    def __init__(self, config: json):
        """init called when ros node starts, sets initial configuration
        for this node and initiates gateway logic listening to mission request
        on port assigned from config file

        Args:
            config (json): Configuration information for colav gateway"""

        super().__init__("colav_gateway")
        self.get_logger().info("colav_gateway node started!")

        self._config = config
        self._topic_namespace = "colav/"
        self._mission_request_topic_name = f"{self._topic_namespace}mission_request"
        mission_request_topic = "colav/mission_request"

        self.ctrl_action_client = ActionClient(
            self, MissionExecutor, "execute_colav_mission"
        )

    async def listen_for_mission_request(self):
        """Listens for incoming protobuf mission_request_pbf2 smg"""
        self.get_logger().info("Initiating COLAV Mission Request Listener.")

        loop = asyncio.get_running_loop()
        host = self._config["endpoint_config"]["mission_request"]["host"]
        port = self._config["endpoint_config"]["mission_request"]["port"]
        transport, protocol = await loop.create_datagram_endpoint(
            lambda: ColavGatewayNode.MissionRequestUDPProtocol(
                self.ctrl_action_client, self.get_logger()
            ),
            local_addr=(host, port),  # Adjust the port as needed
        )
        return transport, protocol

    class MissionRequestUDPProtocol(asyncio.DatagramProtocol):
        """MissionRequestUDPProtocol Is a async udp class that enables real time
        mission requst socket handing converting protobuf mission reqeusts
        to ros message and executing missions for colav.

        Args:
            asyncio (_type_): _description_
        """

        def __init__(self, action_client, logger):
            """"""
            self._action_client = action_client
            self._logger = logger

        def connection_made(self, transport):
            """This method is called when the UDP connection is established."""
            self.transport = transport

        def datagram_received(self, data, addr):
            """Handle incoming datagrams."""
            print(f"Received {data} from {addr}")
            try:
                mission_request_msg = missionRequest_pb2.MissionRequest()
                mission_request_msg.ParseFromString(data)

                ros_mission_msg = MissionRequest()
                ros_mission_msg.mission_tag = mission_request_msg.tag
                ros_mission_msg.mission_sent_timestamp = (
                    mission_request_msg.mission_start_timestamp
                )
                ros_mission_msg.vessel.tag = mission_request_msg.vessel.tag
                ros_mission_msg.vessel.type = (
                    missionRequest_pb2.MissionRequest.Vessel.VesselType.Name(
                        mission_request_msg.vessel.type
                    )
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
                points_list = []
                for (
                    point
                ) in mission_request_msg.vessel.vessel_geometry.polyshape_points:
                    points_list.append(Point32(x=point.x, y=point.y, z=point.z))
                ros_mission_msg.vessel.geometry.polyshape.points = points_list

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
                # ros_mission_msg.mission_goal_acceptance_radius = mission_request_msg.mission_goal_acceptance_radius TODO: Need to add this field to the protobuf msg
                # self._mission_publisher.publish(ros_mission_msg)
                send_goal_future = self._action_client.send_goal_async(
                    ros_mission_msg, feedback_callback=self.feedback_callback
                )
                send_goal_future.add_done_callback(self.goal_response_callback)
            except Exception as e:
                print(e)

        def feedback_callback(self, feedback_msg):
            feedback = feedback_msg.feedback
            self._logger.info(
                f"Controller feedback received, packing feedback in protobuf to send via udp"
            )
            # need to fill out different parts of the message here
            feedback_protobuf = controllerFeedback_pb2.ControllerFeedback()

        def extract_header_and_payload(self, data):
            try:
                # Decode and strip unnecessary characters
                decoded_data = data.decode("ascii").rstrip("\x00").strip()

                # Split the string by the pipe (|) and extract the first three parts
                data_parts = decoded_data.split("|")[:4]

                # Strip any extra spaces from each part and return as a tuple
                data = tuple(part.strip() for part in data_parts)

                return data
            except Exception as e:
                print(f"Error extracting header, invalid udp received: {e}")
                return None

        def error_received(self, exc):
            """Handle errors (optional)."""
            print(f"Error received: {exc}")

        def connection_lost(self, exc):
            """Handle connection loss (optional)."""
            print("Connection lost")


def main():
    logger.info("Parsing colav_gateway config.json.")
    parser = argparse.ArgumentParser(
        description="Script that takes colav_gateway_config.json as arg"
    )
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        help="Path to the config JSON file for colav_gateway.",
        required=True,
    )
    args = parser.parse_args()

    try:
        with open(args.config, "r") as colav_gateway_config_file:
            colav_gateway_config = json.load(colav_gateway_config_file)
            logger.info("colav_gateway config.json successfully read.")
    except Exception as e:
        logger.error(f"Invalid colav_gateway_config.json: {e}")
        exit(1)

    rclpy.init(args=None)
    node = ColavGatewayNode(config=colav_gateway_config)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    executor = rclpy.executors.MultiThreadedExecutor()

    # Run ROS 2 spinning in a separate thread
    loop.run_in_executor(None, lambda: rclpy.spin(node))

    # Now start the UDP server asynchronously
    loop.run_until_complete(node.listen_for_mission_request())

    try:
        loop.run_forever()  # Keeps asyncio running
    except KeyboardInterrupt:
        logger.info("Shutting down colav_gateway.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        loop.close()


if __name__ == "__main__":
    logger.info("Starting colav_gateway application.")
    main()
