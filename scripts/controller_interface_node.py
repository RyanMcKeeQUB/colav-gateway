import rclpy
import rclpy.logging
import os
from colav_interfaces.msg import MissionRequest, Vessel, VesselConstraints, VesselGeometry
from colav_interfaces.msg import CmdVelYaw, ControllerFeedback, ControlMode, ControlStatus
from geometry_msgs.msg import Point32
from colav_interfaces.msg import AgentConfig as ROSAgentConfigUpdateMSG
from geometry_msgs.msg import Polygon
from colav_interfaces.msg import ObstaclesConfig as ROSObstacleConfigUpdateMSG

from colav_protobuf.agentUpdate_pb2 import AgentUpdate as ProtobufAgentConfigUpdate
from colav_protobuf.obstaclesUpdate_pb2 import ObstaclesUpdate as ProtobufObstaclesUpdate
from colav_interfaces.msg import DynamicObstacleConfig
from colav_interfaces.msg import StaticObstacleConfig
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import Header

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
import sys
import asyncio
sys.path.append(os.path.abspath('/home/3507145@eeecs.qub.ac.uk/Documents/ColavProject/colav_ws/src/colav_server/colav_gateway'))
from colav_gateway.utils.udp_socket_utils import setup_udp_socket
from colav_gateway.utils.proto_ros_converter_utils import ProtoToROSUtils
from colav_gateway.utils.msg_validation_utils import validate_mission_request
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import threading
from colav_gateway.utils.ros_proto_converter_utils import ROSTOProtoUtils
logger = rclpy.logging.get_logger("colav_gateway_logger")
workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

class ControllerInterfaceNode(Node):
    """The controller interface node creates an action server which when started
        listens interfaces with agent config updates and obstacle updates multicasting them
        to the ros network for use by hybrid automaton is also starts the hybrid automaton 
        closing the control loop."""
    
    def __init__(
            self, 
            node_namespace: str = "colav_gateway", 
            node_name: str = "controller_interface_node",
            is_thread: bool = False,
            thread_events: dict = None
    ):
        """init function for the ColavGatewayControllerInterface"""
        super().__init__(namespace=node_namespace, node_name=node_name)

        self.declare_parameter('agent_config_host', '0.0.0.0')
        self.declare_parameter('agent_config_port', 7100)

        self.declare_parameter('obstacles_config_host', '0.0.0.0')
        self.declare_parameter('obstacles_config_port', 7200)

        self._agent_config_address = (self.get_parameter('agent_config_host').get_parameter_value().string_value,
                                      self.get_parameter('agent_config_port').get_parameter_value().integer_value)
        self._obstacle_update_address = (self.get_parameter('obstacles_config_host').get_parameter_value().string_value,
                                         self.get_parameter('obstacles_config_port').get_parameter_value().integer_value)

        self._is_thread = is_thread
        if self._is_thread: 
            self._thread_events = thread_events

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.agent_config_publisher = self.create_publisher(
            msg_type=ROSAgentConfigUpdateMSG,
            topic='/agent_config',
            qos_profile=10
        )

        self.obstacles_config_publisher = self.create_publisher(
            msg_type=ROSObstacleConfigUpdateMSG,
            topic='/obstacles_config',
            qos_profile=10
        )
        # # Subscriber to the '/controller_feedback' topic
        self.subscription = self.create_subscription(
            ControllerFeedback,
            '/controller_feedback',
            self.feedback_callback,
            qos_profile
        )
        self._controller_feedback = None
        # self.subscription  # Prevent unused variable warning
        self.start_mission_action_server()

    def feedback_callback(self, msg):
        # Need to create a publisher in here
        self.get_logger().info(f'Received feedback: {msg}')
        self._controller_feedback = ROSTOProtoUtils.parse_controller_feedback(msg = msg)

    def start_mission_action_server(self, server_name:str = "execute_mission"):
        """Starts the action server for the controller interface node."""
        self.get_logger().info(f"Starting {server_name} action server...")
        self._action_server = ActionServer(
            node=self, 
            action_type=MissionExecutor,
            action_name= server_name,
            goal_callback=self._goal_callback,
            execute_callback=self.execute_callback,
            cancel_callback=self._cancel_callback
        )

    def _cancel_callback(self, goal_handle):
        """Action server cancel callback function"""
        self.get_logger().info('received request to cancel goal')
        if self._is_thread:  # if this is a thread. triggers stop_event which will stop the other threads in the gateway
            self._thread_events['stop_event'].set()
        return CancelResponse.ACCEPT       

    async def _start_hybrid_automaton(self, mission_request:MissionRequest, service_timeout:float=2.5, max_service_reconnect_attempts:int = 5):
        """Makes a service request to /start_hybrid_automaton"""
        start_hybrid_automaton_cli = self.create_client(
            StartHybridAutomaton,
            '/start_hybrid_automaton'
        )
        while True:
            if not start_hybrid_automaton_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("/start_hybrid_automaton server not active, attempting to establish connection again....")
                max_service_reconnect_attempts-=1
            if max_service_reconnect_attempts == 0:
                self.get_logger().error("/start_hybrid_automaton service not availabe, closing colav_gateway.")
                self._cancel_callback()
            else: break

        srv_req = StartHybridAutomaton.Request()
        srv_req.mission_request = mission_request
        future = start_hybrid_automaton_cli.call_async(request=srv_req)

        try:
            response = await asyncio.wait_for(future, timeout=service_timeout)
            if response.success:
                self.get_logger().info("/start_hybrid_automaton service has started Hybrid Automaton successfully!")
                return
            else:
                raise Exception(f"/start_hybrid_automaton service request was rejected, due to: {response.message}")
        except asyncio.TimeoutError:
            raise TimeoutError("/start_hybrid_automaton request made but timeout occured.")
        
    def _goal_callback(self, goal_request):
        """Handle the received goal and determine if the hybrid automaton should start."""
        try:
            # self._start_hybrid_automaton(mission_request=validate_mission_request(goal_request.mission_request)) # TODO: uncomments when hybrid automaton is up.
            return GoalResponse.ACCEPT
        except Exception as e:
            self.get_logger().error(f"Mission Rejected: {e}")
        
        return GoalResponse.REJECT

    def _listen_for_agent_config_update(self, stop_event, timeout:float = 10.0):
        """Listen for UDP agent configuration updates."""
        self.get_logger().info('Listening for agent config updates on /agent_config topics...')


        sock = setup_udp_socket(self._agent_config_address, timeout)

        while not stop_event.is_set():  # Allows clean stopping of the loop
            try:
                data, client_address = sock.recvfrom(1024)
                self.get_logger().info(f'Agent Configuration Protobuf received from {client_address}')
                
                agent_update = ProtoToROSUtils.parse_agent_proto(node=self, msg=data)
                if agent_update:
                    self.agent_config_publisher.publish(agent_update)
            except TimeoutError:
                self.get_logger().warning('/agent_config listener socket timeout, Trying again...')
            except Exception as e:
                self.get_logger().warning(e)

        sock.close()  # Close the socket properly when stopping

    def _listen_for_obstacle_config_update(self, stop_event, timeout:float = 10.0):
        """Listen for obstacle updates"""
        self.get_logger().info('Listening for obstacles config updates on /obstacles_config topic...')
        sock = setup_udp_socket(self._obstacle_update_address, timeout)

        while not stop_event.is_set():
            try:
                data, client_address = sock.recvfrom(5020)
                self.get_logger().info(f'Obstacle configuration received from {client_address}')
                
                ros_obstacles_update = ProtoToROSUtils.parse_obstacles_proto(node=self, msg=data)
                if ros_obstacles_update:
                    self.obstacles_config_publisher.publish(ros_obstacles_update)
            except TimeoutError:
                self.get_logger().warning('/obstacles_config listener socket timeout. Trying again...')

    def _receive_controller_feedback(self, goal_handle, stop_event):
        self.get_logger().info('Listening for controller feedback on /controller_feedback topic...')

        while not stop_event.is_set():
            try: 
                feedback_msg = MissionExecutor.Feedback()
                protobuf_ctrl_feedback = ByteMultiArray()
                if self._controller_feedback is not None:
                    serialised_msg = self._controller_feedback.SerializeToString()
                    protobuf_ctrl_feedback.data = [bytes([byte]) for byte in serialised_msg]
                feedback_msg._serialised_protobuf_controller_feedback = protobuf_ctrl_feedback 
                goal_handle.publish_feedback(feedback_msg)  # Publish feedback
                rclpy.spin_once(self, timeout_sec=1)  # Sleep for periodic updates
            except Exception as e:
                self.get_logger().warning(f'{e}')
                stop_event.set()

    def execute_callback(self, goal_handle):
        """Execute the controller feedback loop"""
        self.get_logger().info("Hybrid Automaton successfully started")


        stop_event = threading.Event()
        threads = [
            threading.Thread(target=self._listen_for_agent_config_update, args=(stop_event,)),
            threading.Thread(target=self._listen_for_obstacle_config_update, args=(stop_event,)),
            threading.Thread(target=self._receive_controller_feedback, args=(goal_handle, stop_event, ))
        ]

        for thread in threads:
            thread.start()

        for thread in threads:
            thread.join()

        goal_handle.succeed()  # Mark the action as successfully completed

        result = MissionExecutor.Result()
        result.success = True  # Populate result
        return result
    
# test start
from rclpy.logging import get_logger
from rclpy.executors import MultiThreadedExecutor

logger = get_logger("controller_interface_node")


def main(args=None):

    try:
        rclpy.init(args=args)
        controller_interface_node = ControllerInterfaceNode()

        executor = MultiThreadedExecutor()
        executor.add_node(node = controller_interface_node) 
        executor_thread = threading.Thread(target=executor.spin, daemon=False) # set daemon to true in future to make background worker
        executor_thread.start()

        executor_thread.join()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"Error: {e}")

    controller_interface_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()