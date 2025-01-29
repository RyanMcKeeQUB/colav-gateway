import socket
import rclpy
from rclpy.node import Node
from enum import Enum
import os
import json
from jsonschema import validate, ValidationError
from typing import Tuple


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

        self._sync_mission_req()

    def _sync_mission_req(self):
        # create a socket to listen for mission requests
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as mission_socket:
            mission_socket.bind(("", 7000))
            while True:
                try:
                    mission_socket.settimeout(5)
                    data, _ = mission_socket.recvfrom(1024)
                    msg = data.decode("utf-8")
                    print(f"Received message: {msg}")
                    break  # Exit the loop if a message is received

                except socket.timeout:
                    self._exception_msg = f"{self.__class__.__name__}::{self._listen_for_controller_feedback.__name__} : No response from controller feedback endpoint within timeout period: {str(self._response_timeout)}."
                    self._exception_event.set()
                except Exception as e:
                    self._exception_msg = f"{self.__class__.__name__}::{self._listen_for_controller_feedback.__name__} : An error occurred while listening for controller feedback: {str(e)}"
                    self._exception_event.set()
                    break

            while True:
                conn, addr = mission_socket.accept()
                self.get_logger().info(
                    f"Mission Request socket {(self.HOST, self.REQ_PORT)} connected to be {addr}. Waiting for mission data"
                )
                with conn:

                    mission_data = conn.recv(1024)
                    self._on_mission_req(mission_data)
                    break

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
