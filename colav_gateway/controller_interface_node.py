import rclpy
import rclpy.logging
import os
from colav_interfaces.msg import MissionRequest, Vessel, VesselConstraints, VesselGeometry
from colav_interfaces.msg import CmdVelYaw, ControllerFeedback, ControlMode, ControlStatus
from geometry_msgs.msg import Point32
from colav_interfaces.msg import AgentConfig as ROSAgentConfigUpdateMSG
from geometry_msgs.msg import Polygon
from colav_interfaces.msg import ObstaclesConfig as ROSObstacleConfigUpdateMSG

from agentUpdate_pb2 import AgentUpdate as ProtobufAgentConfigUpdate
from obstaclesUpdate_pb2 import ObstaclesUpdate as ProtobufObstaclesUpdate
from colav_interfaces.msg import DynamicObstacleConfig
from colav_interfaces.msg import StaticObstacleConfig
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from colav_interfaces.action import MissionExecutor
from colav_interfaces.srv import StartHybridAutomaton
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
import json
from typing import Tuple
from std_msgs.msg import MultiArrayDimension
import socket
from rclpy.impl.rcutils_logger import RcutilsLogger
import time

import threading

logger = rclpy.logging.get_logger("colav_gateway_logger")
workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

ROS_NAMESPACE = "colav_gateway"
NODE_NAME = "controller_interface"
ACTION_NAME = "execute_mission"


class ColavGatewayControllerInterface(Node):
    """The controller interface node creates an action server which when started
        listens interfaces with agent config updates and obstacle updates multicasting them
        to the ros network for use by hybrid automaton is also starts the hybrid automaton 
        closing the control loop."""
    
    def __init__(self, config:json, mock:bool = False):
        """init function for the ColavGatewayControllerInterface"""
        super().__init__(namespace=ROS_NAMESPACE, node_name=NODE_NAME)
        
        self._agent_config_address = ("0.0.0.0", 7100)
        self._obstacle_update_address = ("0.0.0.0", 7200)
        self._action_server = ActionServer(
            node=self, 
            action_type=MissionExecutor,
            action_name=ACTION_NAME,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        if not mock:
            self._start_hybrid_automaton_srv_client = self.create_client(
                StartHybridAutomaton,
                '/start_hybrid_automaton'
            )

            while not self._start_hybrid_automaton_srv_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn('Wating for colav_gateway/start_hybrid_automaton service to become available...')

        self.get_logger().info("Planning Action Server Ready")

        
    def listen_for_agent_config_update(self, stop_event):
        """Listen for UDP agent configuration updates."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(self._agent_config_address)  # Ensure this is a tuple (host, port)
        sock.settimeout(1.0)  # Set the timeout once

        self.get_logger().info('COLAV Gateway listening for agent configuration updates.')

        while not stop_event.is_set():  # Allows clean stopping of the loop
            try:
                data, client_address = sock.recvfrom(1024)
                self.get_logger().info(f'Protobuf received from {client_address}')
                print(data)
            except socket.timeout:  # Catch the correct exception
                self.get_logger().warning(
                    'Agent config update listener timeout occurred, possible packet loss'
                )
                continue  # Ensures the loop continues on timeout
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")
                break  # Break on unexpected errors

        sock.close()  # Close the socket properly when stopping        self._obstacle_metadata_address = ("0.0.0.0", 7200)

    def cancel_callback(self, goal_handle):
        """Action server cancel callback function"""
        self.get_logger().info('received request to cancel goal')
        return CancelResponse.ACCEPT
    
    def goal_callback(self, goal_request):
        """Handle the received goal and determine if the hybrid automaton should start."""
        try:
            # Extract and validate the mission request from the goal
            mission_request = self.extract_and_validate_goal_request(goal_request=goal_request, mock=True)
            
            # Attempt to start the hybrid automaton using the mission request
            if self.start_hybrid_automaton(mission_request=mission_request, mock=True):
                self.get_logger().info(
                    '/start_hybrid_automaton request accepted, starting action server feedback loop.'
                )
                return GoalResponse.ACCEPT

        except Exception as e:
            self.get_logger().error(f"Mission Rejected: {e}")
        
        # If Starting Automaton Fails Reject mission
        return GoalResponse.REJECT
        
    def start_hybrid_automaton(self, mission_request:MissionRequest, mock:bool = False) -> bool:
        """Makes a service request to /start_hybrid_automaton"""
        if mock: # Mock allows for mocking of service response
            try:
                return True
            except Exception as e:
                error = "Invalid mission message format"
                raise Exception(
                    f"Error in ColavGatewayControllerInterface::start_hybrid_automaton: "
                    f"The /start_hybrid_automaton request was rejected due to: {error}"
                )
        else:
            pass

    def extract_and_validate_goal_request(self, goal_request, mock:bool = True) -> MissionRequest:
        """Validates the mission request data before sending to hybrid automaton."""
        
        mission_request = None

        if mock:
            mission_request = goal_request.mission_request
        else: 
            mission_request = goal_request.mission_request
        
        self.get_logger().info(
            f"Received goal request for mission: \"{mission_request.mission_tag}\"."
        )
        return mission_request
        
    def listen_for_agent_config_update(self, stop_event):
        """Listen for UDP agent configuration updates."""
        agent_config_publisher = self.create_publisher(
            msg_type=ROSAgentConfigUpdateMSG,
            topic='/agent_config',
            qos_profile=10
        )

        sock = self._setup_udp_socket(self._agent_config_address, 1.0)

        self.get_logger().info('COLAV Gateway listening for agent configuration updates.')

        while not stop_event.is_set():  # Allows clean stopping of the loop
            try:
                data, client_address = sock.recvfrom(1024)
                self.get_logger().info(f'Agent Configuration Protobuf received from {client_address}')
                
                agent_update = self._parse_agent_data(data)
                if agent_update:
                    agent_config_publisher.publish(agent_update)
            except TimeoutError:
                self.get_logger().warning('Protobuf socket timeout. Retrying...')

        sock.close()  # Close the socket properly when stopping

    def _parse_agent_data(self, data:bytes) -> ROSAgentConfigUpdateMSG:
        """Parse agent configuration protobuf to ros"""
        try:
            protobuf_agent_update = ProtobufAgentConfigUpdate()
            protobuf_agent_update.ParseFromString(data)

            ros_agent_update = ROSAgentConfigUpdateMSG(
                agent_tag = protobuf_agent_update.agent_tag,
                pose=self._convert_pose(protobuf_agent_update.state.pose),
                velocity = protobuf_agent_update.state.velocity,
                acceleration = protobuf_agent_update.state.acceleration,
                yaw_rate = protobuf_agent_update.state.yaw_rate,
                timestamp = protobuf_agent_update.timestamp,
                timestep = protobuf_agent_update.timestep
            )
            return ros_agent_update
        except Exception as e:
            self.get_logger().warning(f'Error parsing agent update: {e}')
            return None

    def listen_for_obstacle_config_update(self, stop_event):
        obstacles_config_publisher = self.create_publisher(
            msg_type=ROSObstacleConfigUpdateMSG,
            topic='/obstacles_config',
            qos_profile=10
        )
        
        sock = self._setup_udp_socket(self._obstacle_update_address, 1.0)
        self.get_logger().info('COLAV Gateway listening for obstacle updates.')

        while not stop_event.is_set():
            try:
                data, client_address = sock.recvfrom(5020)
                self.get_logger().info(f'Obstacle configuration received from {client_address}')
                
                ros_obstacles_update = self._parse_obstacle_update(data)
                if ros_obstacles_update:
                    obstacles_config_publisher.publish(ros_obstacles_update)
            except TimeoutError:
                self.get_logger().warning('Protobuf socket timeout. Retrying...')
    
    def _setup_udp_socket(self, address, timeout):
        """Sets up a udp socket"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(address)
        sock.settimeout(timeout)
        return sock

    def _parse_obstacle_update(self, data):
        """Parse Obstacle update received via protobuf and publish it to ros topic"""
        try:
            protobuf_obstacles_update = ProtobufObstaclesUpdate()
            protobuf_obstacles_update.ParseFromString(data)

            ros_obstacles_update = ROSObstacleConfigUpdateMSG(
                dynamic_obstacles=self._convert_dynamic_obstacles(protobuf_obstacles_update.dynamic_obstacles),
                static_obstacles=self._convert_static_obstacles(protobuf_obstacles_update.static_obstacles),
                timestamp=protobuf_obstacles_update.timestamp,
                timestep=protobuf_obstacles_update.timestep
            )
            return ros_obstacles_update
        except Exception as e:
            self.get_logger().warning(f'Error parsing obstacle update: {e}')
            return None

    def _convert_dynamic_obstacles(self, dynamic_obstacles):
        """Convert dynamic obstacles from protobuf to ROS"""
        converted = []
        
        for dynamic_obstacle in dynamic_obstacles:
            ros_dynamic_obstacle = DynamicObstacleConfig(
                id=dynamic_obstacle.id.tag,
                type=ProtobufObstaclesUpdate.ObstacleType.Name(dynamic_obstacle.id.type),
                pose=self._convert_pose(pose=dynamic_obstacle.state.pose),
                velocity=dynamic_obstacle.state.velocity,
                yaw_rate=dynamic_obstacle.state.yaw_rate,
                geometry=self._convert_geometry(dynamic_obstacle.geometry),
                safety_radius=dynamic_obstacle.geometry.acceptance_radius
            )
            converted.append(ros_dynamic_obstacle)
        return converted
    
    def _convert_pose(self, pose) -> Pose:
        """Converts a protobuf pose to ros msg"""
        return Pose(
            position=Point(
                x = pose.position.x,
                y = pose.position.y,
                z = pose.position.z
            ),
            orientation=Quaternion(
                x = pose.orientation.x,
                y = pose.orientation.y,
                z = pose.orientation.z,
                w = pose.orientation.w
            )
        )

    def _convert_static_obstacles(self, static_obstacles):
        """Convert protobuf static obstacles to ROS static obstacles"""
        converted = []
        for static_obstacle in static_obstacles:
            ros_static_obstacle = StaticObstacleConfig(
                id=static_obstacle.id.tag,
                type=ProtobufObstaclesUpdate.ObstacleType.Name(static_obstacle.id.type),
                pose=self._convert_pose(static_obstacle.pose),
                geometry=self._convert_geometry(static_obstacle.geometry),
                safety_radius=static_obstacle.geometry.acceptance_radius
            )
            converted.append(ros_static_obstacle)
        return converted

    def _convert_geometry(self, geometry):
        """Converts polybuf geometry to ros msg geometry"""
        return Polygon(points = [Point32(x=point.position.x, y=point.position.y, z=point.position.z) for point in geometry.polyshape_points])

    def listen_for_controller_feedback(self, stop_event):
        # Create the subscription once outside the loop
        self.create_subscription(
            ControllerFeedback,
            '/controller_feedback',
            self.controller_feedback_callback,
            10
        )
        self.get_logger().info('Listening for feedback on /controller_feedback')

        # Continuously listen for feedback without reinitializing the subscription
        while not stop_event.is_set():
            # Just sleep, the subscription is always active
            time.sleep(1.0)

        self.get_logger().info('Stopped listening for feedback.')


    def controller_feedback_callback(self, msg):
        self.get_logger().info(f'msg received: {msg}')

    async def execute_callback(self, goal_handle):
        """Execute the controller feedback loop"""
        self.get_logger().info("Hybrid Automaton successfully started")
        stop_event = threading.Event()
        threads = [
            threading.Thread(target=self.listen_for_agent_config_update, args=(stop_event,)),
            threading.Thread(target=self.listen_for_obstacle_config_update, args=(stop_event,)),
            threading.Thread(target=self.listen_for_controller_feedback, args=(stop_event,))  # Possible copy-paste mistake?
        ]

        for thread in threads:
            thread.start()

        for thread in threads:
            thread.join()
    
    # async def send_start_hybrid_automaton_request(self, mission_request: MissionRequest) -> Tuple[bool, str]:
    #     """Send an asynchronous service request to start the hybrid automaton"""
    #     if not self._start_hybrid_automaton_srv_client.wait_for_service(timeout_sec=3.0):
    #         self.get_logger().error("Hybrid Automaton service is unavailable.")
    #         return False, "Service unavailable"

    #     mission_request_srv = StartHybridAutomaton.Request()
    #     mission_request_srv.mission_request = mission_request
    #     future = self._start_hybrid_automaton_srv_client.call_async(mission_request_srv)

    #     try:
    #         response = await future  # Await the future instead of blocking
    #         self.get_logger().info(f"Hybrid Automaton Started: {response.success}, Message: {response.message}")
    #         return response.success, response.message
    #     except Exception as e:
    #         self.get_logger().error(f"Service call failed: {str(e)}")
    #         return False, f"Error: {str(e)}"

def main(args=None):
    rclpy.init(args=args)
    node = ColavGatewayControllerInterface(config=None, mock=True) # We are mocking for testing 
    try: 
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    logger.info("Colav Planning Action Server started")
    main()