import rclpy
import rclpy.logging
import os
from colav_interfaces.msg import MissionRequest, Vessel, VesselConstraints, VesselGeometry
from colav_interfaces.msg import CmdVelYaw, ControllerFeedback, ControlMode, ControlStatus

from colav_interfaces.msg import AgentConfig as ROSAgentConfigUpdateMSG
from colav_interfaces.msg import ObstaclesConfig as ROSObstacleConfigUpdateMSG

from agentUpdate_pb2 import AgentUpdate as ProtobufAgentCOnfigUpdate
from obstacleUpdate_pb2 import ObstacleUpdate as ProtobufObstaclesUpdate

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

    def __init__(self, config:json, mock:bool = False):
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

        # TODO: Need to create client to /agent_configuration first
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(self._agent_config_address)  # Ensure this is a tuple (host, port)
        sock.settimeout(1.0)  # Set the timeout once

        self.get_logger().info('COLAV Gateway listening for agent configuration updates.')

        while not stop_event.is_set():  # Allows clean stopping of the loop
            try:
                data, client_address = sock.recvfrom(1024)
                self.get_logger().info(f'Agent Configuration Protobuf received from {client_address}')
                
                try: 
                    agent_config_protobuf = ProtobufAgentCOnfigUpdate()
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
        # target = 10
        # x = 0

        # while x < 10: 
        #     self.get_logger().info("listening for agent_config 1 is running")
        #     x+=1

    # Example function for Task 2
    def listen_for_obstacle_config_update(self, stop_event):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(self._obstacle_update_address)  # Ensure this is a tuple (host, port)
        sock.settimeout(1.0)  # Set the timeout once

        self.get_logger().info('COLAV Gateway listening for obstacles updates.')

        while not stop_event.is_set():  # Allows clean stopping of the loop
            try:
                data, client_address = sock.recvfrom(1024)
                self.get_logger().info(f'Agent Configuration Protobuf received from {client_address}')
                
                try: 
                    pass
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



        # # UDP Listener started successfully
        # self.stop_event = asyncio.Event()
        
        # # Ensure the event loop is running before creating tasks
        # loop = asyncio.get_event_loop()

        # # Check if the event loop is running and schedule tasks
        # if loop.is_running():
        #     tasks = [
        #         asyncio.ensure_future(self.listen_for_agent_config_update(self.stop_event)),
        #         asyncio.ensure_future(self.listen_for_obstacles_update(self.stop_event)),
        #         asyncio.ensure_future(self.controller_feedback_listener(self.stop_event))
        #     ]
            
        #     # Await the tasks concurrently
        #     await asyncio.gather(*tasks)
        # else:
        #     self.get_logger().error("No running event loop found")

        # feedback = MissionExecutor.Feedback()
        # from std_msgs.msg import ByteMultiArray 
        # protobuf_controller_msg = ByteMultiArray()

        # element_1 = MultiArrayDimension()
        # element_1.label = "1"
        # element_1.size = 2
        # element_1.stride = 1

        # protobuf_controller_msg.layout.dim = [element_1]
        # protobuf_controller_msg.data = [bytes(1), bytes(2)]

        # feedback.serialised_protobuf_controller_feedback = protobuf_controller_msg

        # goal_handle.publish_feedback(feedback)

        # # # Function to periodically send feedback
        # # def feedback_timer_callback():
        # #     if not goal_handle.is_cancel_requested:
        # #         goal_handle.publish_feedback(feedback)
        # #     else:
        # #         self.timer.cancel()  # Stop the timer when the goal is canceled

        # # # Create a timer to call the feedback callback every second
        # # self.timer = self.create_timer(1.0, feedback_timer_callback)  # 1 second interval

        # while not goal_handle.is_cancel_requested:
        #     # Here, we just wait for the goal handle to be canceled
        #     goal_handle.publish_feedback(feedback)
        #     time.sleep(1)

        # goal_handle.succeed()
        # else:
        #     self.get_logger().warning(f"Hybrid Automaton failed to start with mission request")
            # goal_handle.abort()

    # Listener function example
    # async def listen_for_agent_config_update(self, stop_event):
    #     while not stop_event.is_set():
    #         # Your task logic here
    #         await asyncio.sleep(0.1)  # Simulate work
    #     self.get_logger().info("Agent config update listener stopped")

    # # Similar structure for other listener functions
    # async def listen_for_obstacles_update(self, stop_event):
    #     while not stop_event.is_set():
    #         # Your task logic here
    #         await asyncio.sleep(0.1)  # Simulate work
    #     self.get_logger().info("Obstacles update listener stopped")

    # async def controller_feedback_listener(self, stop_event):
    #     while not stop_event.is_set():
    #         # Your task logic here
    #         await asyncio.sleep(0.1)  # Simulate work
    #     self.get_logger().info("Controller feedback listener stopped")

    # async def controller_feedback_listener(self):
    #     """Listens for controller feedback via ros topic and converts it to protobuf byte message to be returned as action feedback for colav gateway,"""
    #     pass

    # async def listen_for_obstacles_update(self):
    #     """Starts the UDP listener for obstacle updates"""
    #     self.get_logger().info("Initiation COLAV Obstacle Update listener")
    #     loop = asyncio.get_running_loop()
    #     transport, protocol = await loop.create_datagram_endpoint(
    #         lambda: self.ObstacleMetadataUDPProtocol(self, self.get_logger()),
    #         local_addr=self._obstacle_metadata_address,
    #     )
    #     return transport, protocol

    # async def listen_for_agent_config_update(self):
    #     """Starts the UDP listener for agent configuration updates."""
    #     self.get_logger().info("Initiating COLAV Agent Configuration updates.")
    #     loop = asyncio.get_running_loop()
    #     transport, protocol = await loop.create_datagram_endpoint(
    #         lambda: self.AgentConfigUDPProtocol(self, self.get_logger()),
    #         local_addr=self._agent_config_address,
    #     )
    #     return transport, protocol

    class AgentConfigUDPProtocol(asyncio.DatagramProtocol):
        """Handles incoming UDP agent configurations and forwards them to the COLAV system"""
        def __init__(self, logger: RcutilsLogger):
            self._logger = logger
            self.agent_config_publisher = self.create_publisher(
                ROSAgentConfigUpdateMSG,
                '/agent_config',
                10
            )

        def connection_made(self, transport: asyncio.DatagramTransport):
            self.transport = transport
        
        def datagram_received(self, data, addr):
            self._logger.info(f"Received data from {addr}")
            ros_agent_config_msg = self.protobuf_agent_config_to_ros_msg(data)
            self.publish_ros_msg(ros_agnet_config_msg=ros_agent_config_msg)
        
        def protobuf_agent_config_to_ros_msg(self, data):
            agent_config_msg = ProtobufAgentCOnfigUpdate()
            agent_config_msg.ParseFromString(data)
            return self.convert_to_ros_msg(agent_config_msg)
        
        def convert_to_ros_msg(self, agent_config_msg):
            """Convert a protobuf agent configuration message to a ROS AgentConfig message"""
            pass

        def publish_ros_msg(self, ros_agent_config_msg): 
            """publishes the ros message"""
            self.agent_config_publisher.publish(
                ros_agent_config_msg
            )

        def connection_lost(self, exc):
            self._logger.warning("UDP Connection lost for agent config listener.")

    class ObstacleMetadataUDPProtocol(asyncio.DatagramProtocol):
        """Handles incoming UDP obstacle configurations and forwards them to the COLAV system"""
        def __init__(self, logger: RcutilsLogger):
            self._logger = logger
            self.agent_config_publisher = self.create_publisher(
                ROSObstacleConfigUpdateMSG,
                '/obstacle_config',
                10
            )

        def connection_made(self, transport: asyncio.DatagramTransport):
            self.transport = transport
        
        def datagram_received(self, data, addr):
            self._logger.info(f"Received data from {addr}")
            ros_obstacle_update = self.protobuf_obstacle_config_to_ros_msg(data)
            self.publish_ros_msg(ros_agnet_config_msg=ros_obstacle_update)
        
        def protobuf_obstacle_config_to_ros_msg(self, data):
            agent_config_msg = ProtobufAgentCOnfigUpdate()
            agent_config_msg.ParseFromString(data)
            return self.convert_to_ros_msg(agent_config_msg)
        
        def convert_to_ros_msg(self, agent_config_msg):
            """Convert a protobuf agent configuration message to a ROS AgentConfig message"""
            pass

        def publish_ros_msg(self, ros_agent_config_msg): 
            """publishes the ros message"""
            self.agent_config_publisher.publish(
                ros_agent_config_msg
            )

        def connection_lost(self, exc):
            self._logger.warning("UDP Connection lost for agent config listener.")


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


        # Ok Here I am going to make a service request to /state_hybrid_automaton 
        # if the service returns true then I will start my asyncronous publishing of global updates
        # for the hybrid automaton
        # self.get_logger().info(f"Executing goal: {goal_handle.request.mission_tag}")

        # # feedback_msg = ControllerFeedback()
        # self.get_logger().info(f"Received Mission Request: {goal_handle.request}")

    # async def upd_listener(self):
    #     self.get_logger().info(f"Listening for UDP messages for agent config and obstacle metadata on: "
    #                         f"\n\tAgent State Updates: ({self.agent_update_udp_host}, {self.agent_update_udp_port})"
    #                         f"\n\tObstacle Metadata: ({self.obstacle_metadata_udp_host}, {self.obstacle_metadata_udp_port})")
    #     self.agent_config_udp_listener()
        

    # async def agent_config_udp_listener(self):
    #     """Listen for incoming UDP Protobuf messages to transmit to Hybrid Automaton"""
    #     loop = asyncio.get_running_loop()
    #     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #     sock.bind((self.agent_update_udp_host, self.agent_update_udp_port))
    
    # async def obstacle_metadata_udp_listener(self):
    #     """Listen for incominb UDP Protobuf messages to transmit to Hybrid Automaton"""
    #     loop = asyncio.get_running_loop()
    #     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #     sock.bind((self.obstacle_metadata_udp_host, self.obstacle_metadata_udp_port))

    # def feedback_callback(self, msg):
    #     self.get_logger().info(f"Received ROS Feedback: {msg.data}")
    #     self.feedback.controller_feedback = msg.data

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