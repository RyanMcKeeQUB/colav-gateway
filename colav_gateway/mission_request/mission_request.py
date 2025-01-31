import socket
import rclpy
from rclpy.node import Node
from enum import Enum
import os
import json
from jsonschema import validate, ValidationError
from typing import Tuple, Optional

SCHEMA_DIR = os.path.join(os.path.dirname(__file__), "schemas/")
ROS2_SRV_ROUTING_TABLE = {
    "mission_osm_generator": "/colav/colav_map/generate_mission_osm",
    "mission_global_plan": "/colav/global_path_planner/generate_safe_waypoints",
}


def load_schema(file_name):
    file_path = os.path.join(SCHEMA_DIR, file_name)
    try:
        with open(file_path, "r") as f:
            return json.load(f)
    except FileNotFoundError:
        raise FileNotFoundError(f"Schema file '{file_name}' not found.")
    except json.JSONDecodeError:
        raise ValueError(f"Failed to decode JSON from '{file_name}'.")


class MissionRequestNode(Node):

    def __init__(
        self,
        req_address: Tuple[str, int],  # local_address (host, port)
        res_address: Tuple[str, int],  # remote_address (host, port)
        tmp_osm_dir: str = "",
        workspace_dir: str = "",
    ):
        super().__init__("colav_gateway_mission_request_node")
        self.request_address = req_address
        self.response_address = res_address

        self.ASU_MISSION_REQUEST_SCHEMA = load_schema(
            os.path.join(workspace_dir, "schemas/", "ASU_MISSION_REQUEST_SCHEMA.json")
        )
        self.ASU_MISSION_REQUEST_RESPONSE_SCHEMA = load_schema(
            os.path.join(
                workspace_dir, "schemas/", "ASU_MISSION_REQUEST_RESPONSE_SCHEMA.json"
            )
        )
        self.tmp_osm_dir = tmp_osm_dir
        self.ws_dir = workspace_dir

        self._listen_for_mission_request()

    class SocketBindError(Exception):
        """Custom exception for socket binding errors."""
        def __init__(self, address, message="Socket is already bound to the address"):
            self.address = address
            self.message = message
            super().__init__(f"{message}: {address}")
    
    def create_socket(protocol: str, bind_address: Optional[Tuple[str, int]] = None):
        """Creates a socket with the provided protocol and address."""
        sock = None
        
        try: 
            if protocol == 'UDP':
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            elif protocol == 'TCP':
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            else:
                print('Invalid protocol for socket.')
                return sock
            
            if bind_address is not None: 
                    sock.bind((bind_address[0], bind_address[1]))
            
            return sock
        
        except socket.error as e:
            if e.errno == socket.errno.EADDRINUSE:
                raise MissionRequestNode.SocketBindError(bind_address) from e
            else:
                raise e

    def _listen_for_mission_request(self) -> bool: # NO THREAD FOR THIS METHOD
        """
        listens for a mission request, if a mission request is received. This function starts another thread which will being listening 
        for global updates and publishing them to their respective topics.

        Args:
            mission_request_msg (str): The mission request message to be sent.

        Raises:
            ValueError: If the response indicates that the mission request contained invalid parameters.
            TimeoutError: If the maximum number of mission request attempts is exceeded.
            BrokenPipeError: If the connection was closed by the other end.
            Exception: For any other exceptions that occur during the process.
        """
        mission_started = False
        request_sock = MissionRequestNode.create_socket(protocol=self.mission_request_endpoint['protocol'].upper())
        response_sock = MissionRequestNode.create_socket(bind_address=self.mission_response_endpoint['address'], protocol=self.mission_request_endpoint['protocol'].upper())

        try:
            # serialized_msg = msgpack.packb(json.loads(msg)) # cant deserialise in matlab because imports not allowed in matlab system blocks
            encoded_msg = self._encode_udp_msg(msg=msg)
            request_attempts = 0
            print(f"[INFO] Publishing mission request to endpoint {self.mission_request_endpoint['address']} with the following message:\n{msg}")
            
            while request_attempts <= self.MAX_ATTEMPTS:
                # Send the mission request message
                if self.mission_request_endpoint['protocol'].upper() == 'UDP':
                    request_sock.sendto(encoded_msg, (self.mission_request_endpoint['address'][0], self.mission_request_endpoint['address'][1]))

                # Listen for a response
                response_sock.settimeout(self.INTERVAL)
                try:
                    data, addr = response_sock.recvfrom(self.BUFFER_SIZE)
                    response_msg = extract_udp_msg(data)
                    print(f"Received response from {addr}: {response_msg}")
                    if response_msg == "MISSION_REQUEST_RECEIVED:STARTING":
                        print(f"[INFO] Mission request received. Motion planner is starting the mission.\n{'-'*100}\n")
                        mission_started = True
                        break
                    elif response_msg == "MISSION_REQUEST_RECEIVED:INVALID_PARAMS":
                        raise ValueError(f"{response_msg}: The mission request contained invalid parameters.")
                    else:
                        print(f"Unknown response received: {response_msg}, Trying again....")
                except socket.timeout:
                    print(f"No response to mission request publish attempt: {str(request_attempts + 1)}/{str(self.MAX_ATTEMPTS)}, retrying...")
                finally:
                    request_attempts += 1

            if request_attempts >= self.MAX_ATTEMPTS:
                raise TimeoutError("Exceeded maximum number of Mission Request Attempts.")
        except BrokenPipeError:
            print("BrokenPipeError: The connection was closed by the other end.")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            request_sock.close()
            response_sock.close()
            return mission_started

    def _on_mission_req(self, mission_data):
        if not mission_data:
            self.get_logger().info("Error: Mission Request data not received.")
            return
        else:
            # extract mission request data
            asu_mission_request_json = self._packet_to_json(mission_data)

            # extract mission data components
            planning_problem_json = asu_mission_request_json["planning_problem"]
            agent_config_json = asu_mission_request_json["agent_config"]

            is_problem_valid = (
                True  # self._validate_planning_problem(planning_problem_json)
            )
            is_agent_config_valid = (
                True  # self._validate_agent_config(agent_config_json)
            )

            # validate mission data components
            if is_problem_valid and is_agent_config_valid:
                is_map_created = self._create_mission_map_client(
                    agent_config_json["vessel_configuration"],
                    (
                        planning_problem_json["goal_waypoint"]["latitude"],
                        planning_problem_json["goal_waypoint"]["longitude"],
                    ),
                )
                if is_map_created == True:
                    self.get_logger().info(
                        "Global map generated... starting global map waypoint generation."
                    )
                    return True
                else:
                    self.get_logger().info("Invalid map; could not be extracted.")
                    return False
            else:
                self.get_logger().error(
                    "Invalid planning problem or agent configuration send over tcp"
                )
                return False

    def _create_mission_map_client(
        self,
        agent_position: Tuple[str, str] = ("40.7128N", "74.0060E"),
        goal_waypoint: Tuple[float, float] = ("45.1234N", "76.01235E"),
    ):
        _map_client = self.create_client(
            "colav/map/create_global_map_srv",
        )

    def _validate_planning_problem(self, data: json) -> bool:
        return True

    def _validate_agent_config(self, data: json) -> bool:
        return True

    def _packet_to_json(self, data: bytes) -> json:
        try:
            decoded_data = data.decode("utf-8")
            try:
                asu_mission_request_json = json.loads(decoded_data)
                validate(
                    instance=asu_mission_request_json,
                    schema=self.ASU_MISSION_REQUEST_SCHEMA,
                )
                self.get_logger().info(
                    "asu_mission_request byte packet data converted to json and schema validated"
                )
                return asu_mission_request_json
            except json.JSONDecodeError as e:
                self.get_logger().error(str(e.msg))
                self.get_logger().error(f"Failed to decode Mission Request packet")
                return None
        except UnicodeDecodeError as e:
            self.get_logger().error(f"asu_mission_request decoding failed {e.msg}")
            return None

    def _validate_planning_problem(self, data: json) -> bool:
        pass

    def _validate_agent_config(self, data: json) -> bool:
        pass

    def _make_global_map_req(self, planning_problem: json, agent_config: json):
        pass

    def _make_global_plan(self, planning_problem: json, agent_config: json):
        pass

    def _make_sub_mission_map(
        self,
        sub_mission_id,
        sub_mission_waypoint: Tuple[float, float],
        map_bloat: float = 1.15,
    ):
        pass
