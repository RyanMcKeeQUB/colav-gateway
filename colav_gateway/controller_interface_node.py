import rclpy
import rclpy.logging
import os
from colav_interfaces.msg import MissionRequest, Vessel, VesselConstraints, VesselGeometry
from colav_interfaces.msg import CmdVelYaw, ControllerFeedback, ControlMode, ControlStatus
from geometry_msgs.msg import Point32
from colav_interfaces.msg import AgentConfig as ROSAgentConfigUpdateMSG
from colav_interfaces.msg import ObstaclesConfig as ROSObstacleConfigUpdateMSG

from agentUpdate_pb2 import AgentUpdate as ProtobufAgentConfigUpdate
from obstaclesUpdate_pb2 import ObstaclesUpdate as ProtobufObstaclesUpdate

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
import asyncio

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

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(self._agent_config_address)  # Ensure this is a tuple (host, port)
        sock.settimeout(1.0)  # Set the timeout once

        self.get_logger().info('COLAV Gateway listening for agent configuration updates.')

        while not stop_event.is_set():  # Allows clean stopping of the loop
            try:
                data, client_address = sock.recvfrom(1024)
                self.get_logger().info(f'Agent Configuration Protobuf received from {client_address}')
                
                try: 
                    agent_config_protobuf = ProtobufAgentConfigUpdate()
                    agent_config_protobuf.ParseFromString(data)

                    ros_agent_config = ROSAgentConfigUpdateMSG()
                    ros_agent_config.agent_tag = agent_config_protobuf.agent_tag
                    ros_agent_config.pose.position.x = agent_config_protobuf.state.pose.position.x
                    ros_agent_config.pose.position.y = agent_config_protobuf.state.pose.position.y
                    ros_agent_config.pose.position.z = agent_config_protobuf.state.pose.position.z

                    ros_agent_config.pose.orientation.x = agent_config_protobuf.state.pose.orientation.x
                    ros_agent_config.pose.orientation.y = agent_config_protobuf.state.pose.orientation.y
                    ros_agent_config.pose.orientation.z = agent_config_protobuf.state.pose.orientation.z
                    ros_agent_config.pose.orientation.w = agent_config_protobuf.state.pose.orientation.w

                    # ros_agent_config.orientation = #TODO: Need to add orientation
                    ros_agent_config.velocity = agent_config_protobuf.state.velocity
                    ros_agent_config.yaw_rate = agent_config_protobuf.state.yaw_rate
                    ros_agent_config.acceleration = agent_config_protobuf.state.acceleration

                    ros_agent_config.timestamp = agent_config_protobuf.timestamp
                    ros_agent_config.timestep = agent_config_protobuf.timestep

                    agent_config_publisher.publish(ros_agent_config)
                except Exception as e:
                    self.get_logger().warning(f'Issue deserialising agent config packet received.')

            except socket.timeout:  # Catch the correct exception
                self.get_logger().warning(
                    'Agent config update listener timeout occurred, possible packet loss'
                )
                continue  # Ensures the loop continues on timeout
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")
                break  # Break on unexpected errors

        sock.close()  # Close the socket properly when stopping

    # Example function for Task 2
    def listen_for_obstacle_config_update(self, stop_event):

        obstacles_config_publisher = self.create_publisher(
            msg_type=ROSObstacleConfigUpdateMSG,
            topic='/obstacles_config',
            qos_profile=10
        )
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(self._obstacle_update_address)  # Ensure this is a tuple (host, port)
        sock.settimeout(1.0)  # Set the timeout once

        self.get_logger().info('COLAV Gateway listening for obstacles updates.')

        while not stop_event.is_set():  # Allows clean stopping of the loop
            try:
                data, client_address = sock.recvfrom(5020)
                self.get_logger().info(f'Agent Configuration Protobuf received from {client_address}')
                
                try: 
                    protobuf_obstacles_update = ProtobufObstaclesUpdate()
                    protobuf_obstacles_update.ParseFromString(data)

                    ros_obstacles_update = ROSObstacleConfigUpdateMSG()

                    # need to iterate through protobuf obstacles adding them to ros_obstacles_update
                    dynamic_obstacles = []
                    from colav_interfaces.msg import DynamicObstacleConfig
                    for dynamic_obstacle in protobuf_obstacles_update.dynamic_obstacles:
                        ros_dynamic_obstacle = DynamicObstacleConfig()
                        ros_dynamic_obstacle.id = dynamic_obstacle.id.tag
                        ros_dynamic_obstacle.type = ProtobufObstaclesUpdate.ObstacleType.Name(dynamic_obstacle.id.type) 
                        ros_dynamic_obstacle.pose.position.x = dynamic_obstacle.state.pose.position.x
                        ros_dynamic_obstacle.pose.position.y = dynamic_obstacle.state.pose.position.y
                        ros_dynamic_obstacle.pose.position.z = dynamic_obstacle.state.pose.position.z

                        ros_dynamic_obstacle.pose.orientation.x = dynamic_obstacle.state.pose.orientation.x
                        ros_dynamic_obstacle.pose.orientation.y = dynamic_obstacle.state.pose.orientation.y
                        ros_dynamic_obstacle.pose.orientation.z = dynamic_obstacle.state.pose.orientation.z
                        ros_dynamic_obstacle.pose.orientation.w = dynamic_obstacle.state.pose.orientation.w
                        
                        ros_dynamic_obstacle.velocity = dynamic_obstacle.state.velocity
                        ros_dynamic_obstacle.yaw_rate = dynamic_obstacle.state.yaw_rate

                        # geometry 
                        points = []
                        for point in dynamic_obstacle.geometry.polyshape_points:
                            ros_point = Point32()
                            ros_point.x = point.position.x
                            ros_point.y = point.position.y
                            ros_point.z = point.position.z
                            points.append(ros_point)

                        ros_dynamic_obstacle.geometry.points = points
                        ros_dynamic_obstacle.safety_radius = dynamic_obstacle.geometry.acceptance_radius
                        dynamic_obstacles.append(ros_dynamic_obstacle)
                        # 

                    static_obstacles = []
                    for static_obstacle in protobuf_obstacles_update.static_obstacles:
                        from colav_interfaces.msg import StaticObstacleConfig
                        ros_static_obstacle = StaticObstacleConfig()    
                        ros_static_obstacle.id = static_obstacle.id.tag
                        ros_static_obstacle.type = ProtobufObstaclesUpdate.ObstacleType.Name(static_obstacle.id.type)
                        ros_static_obstacle.pose.position.x = static_obstacle.pose.position.x
                        ros_static_obstacle.pose.position.y = static_obstacle.pose.position.y
                        ros_static_obstacle.pose.position.z = static_obstacle.pose.position.z

                        ros_static_obstacle.pose.orientation.x =  static_obstacle.pose.orientation.x
                        ros_static_obstacle.pose.orientation.y =  static_obstacle.pose.orientation.y
                        ros_static_obstacle.pose.orientation.z =  static_obstacle.pose.orientation.z
                        ros_static_obstacle.pose.orientation.w =  static_obstacle.pose.orientation.w

                        points = []
                        for point in static_obstacle.geometry.polyshape_points:
                            ros_point = Point32()
                            ros_point.x = point.position.x
                            ros_point.y = point.position.y
                            ros_point.z = point.position.z
                            points.append(ros_point)

                        ros_static_obstacle.geometry.points = points
                        ros_static_obstacle.safety_radius = static_obstacle.geometry.acceptance_radius
                        # add geometry

                        static_obstacles.append(ros_static_obstacle)

                    ros_obstacles_update.dynamic_obstacles = dynamic_obstacles
                    ros_obstacles_update.static_obstacles = static_obstacles
                    
                    ros_obstacles_update.timestamp = protobuf_obstacles_update.timestamp
                    ros_obstacles_update.timestep = protobuf_obstacles_update.timestep

                    obstacles_config_publisher.publish(ros_obstacles_update)
                except Exception as e: 
                    self.get_logger().warning(f"Error parsing data received...")
            except TimeoutError as e:
                self.get_logger().warning(f"Agent configuration protobuf socket timeout.... Trying again.")

    def listen_for_controller_feedback(self, step_event):
        # This function is going to provide the feedback for the action server.
        # controller_feedback_client = self.create_client(
        #     ControllerFeedback,
        #     '/controller_feedback',
        #     10
        # )
        pass

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