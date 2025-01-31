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

from colav_interfaces.msg import Mission
import numpy as np
from geometry_msgs.msg import Point32

logger = rclpy.logging.get_logger("colav_gateway_logger")
workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class ColavGatewayNode(Node):

    def __init__(self, config: json):
        super().__init__("colav_gateway")
        self.get_logger().info("colav_gateway node started!")
        # self._init_colav_topics()

        self.host = "0.0.0.0"
        self.port = 7000
        self.config = config  # Store the config for later use

        mission_request_topic = "colav/mission_request"
        self._mission_publisher = self.create_publisher(
            Mission, 
            mission_request_topic, 
            10
        )

    async def listen_for_mission_request(self):
        """Starts listening for incoming UDP packets"""
        self.get_logger().info("Initiating COLAV Mission Request Listener.")

        loop = asyncio.get_running_loop()
        transport, protocol = await loop.create_datagram_endpoint(
            lambda: ColavGatewayNode.UDPProtocol(self._mission_publisher), 
            local_addr=('0.0.0.0', 9999)  # Adjust the port as needed
        )
        return transport, protocol
    
    class UDPProtocol(asyncio.DatagramProtocol):

        class ColavUDPProtocols(Enum):
            MISSION_REQUEST = "MISSION_REQUEST"
            OBSTACLES_UPDATE = "OBSTACLES_UPDATE"
            AGENT_UPDATE = "AGENT_UPDATE"

            @staticmethod
            def is_protocol(protocol:str):
                match protocol:
                    case "MISSION_REQUEST":
                        return ColavGatewayNode.UDPProtocol.ColavUDPProtocols.MISSION_REQUEST
                    case "OBSTACLES_UPDATE":
                        return ColavGatewayNode.UDPProtocol.ColavUDPProtocols.OBSTACLES_UPDATE
                    case "AGENT_UPDATE":
                        return ColavGatewayNode.UDPProtocol.ColavUDPProtocols.AGENT_UPDATE 
                    case _:
                        raise Exception('Invalid UDP packet procotol')

        def __init__(self, mission_publisher):
            self._mission_publisher = mission_publisher

        def connection_made(self, transport):
            """This method is called when the UDP connection is established."""
            self.transport = transport


        def datagram_received(self, data, addr):
            """Handle incoming datagrams."""
            print(f"Received {data} from {addr}")
            try: 
                header = self.extract_header_and_payload(data)
                print (header[0])
                print (header[1])
                print (header[2])
                print (header[3])

                if (header[0] == 'MISSION_REQUEST'):
                    try:    
                        json_payload = json.loads(header[-1])

                        mission_msg = Mission()
                        mission_msg.mission_tag = json_payload["tag"]
                        mission_msg.mission_sent_timestamp = json_payload["timestamp"]
                        mission_msg.vessel.tag = json_payload["vessel"]["tag"]
                        mission_msg.vessel.type = json_payload["vessel"]["type"]
                        mission_msg.vessel.dynamic_constraints.max_acceleration = float(json_payload["vessel"]["constraints"]["max_acceleration"])
                        mission_msg.vessel.dynamic_constraints.max_deceleration = float(json_payload["vessel"]["constraints"]["max_deceleration"])
                        mission_msg.vessel.dynamic_constraints.max_velocity = float(json_payload["vessel"]['constraints']["max_velocity"])
                        mission_msg.vessel.dynamic_constraints.min_velocity = float(json_payload["vessel"]['constraints']['min_velocity'])
                        mission_msg.vessel.dynamic_constraints.max_yaw_rate = float(json_payload['vessel']['constraints']['max_yaw_rate'])

                        mission_msg.vessel.geometry.acceptance_radius = float(json_payload["vessel"]["geometry"]["acceptance_radius"])
                        
                        mission_msg.vessel.geometry.polyshape.points = [Point32(x=float(point[0]), y=float(point[1]), z=float(point[2])) for point in np.array(json_payload["vessel"]["geometry"]["polyshape_points"], dtype=float)]

                        init_position = np.array(json_payload["mission_init_position"], dtype=float)
                        mission_msg.mission_init_position = Point32(x=init_position[0], y=init_position[1], z=init_position[2])
                        goal_position = np.array(json_payload["mission_goal_position"], dtype=float)
                        mission_msg.mission_goal_position = Point32(x=goal_position[0], y=goal_position[1], z=goal_position[2]) 

                        mission_msg.mission_goal_acceptance_radius = float(json_payload["mission_goal_acceptance_radius"])
                        # mission_msg.vessel.
                        self._mission_publisher.publish(mission_msg)

                        # Now trigger the mission response. Mission request will be mission starting UDP message since the mission was sent successfully.
                    except Exception as e:
                        raise Exception(f'Error: {e}')
            except Exception as e:
                print (e)
                # Trigger mission response being invalid mission request received.

        def extract_header_and_payload(self, data):
            try:
                # Decode and strip unnecessary characters
                decoded_data = data.decode("ascii").rstrip("\x00").strip()
                
                # Split the string by the pipe (|) and extract the first three parts
                data_parts = decoded_data.split('|')[:4]
                
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
