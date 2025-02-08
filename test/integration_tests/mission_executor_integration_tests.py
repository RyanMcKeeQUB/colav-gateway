import unittest
from unittest.mock import MagicMock
import rclpy
from rclpy.node import Node
from colav_interfaces.action import MissionExecutor
from missionRequest_pb2 import MissionRequest
from missionResponse_pb2 import MissionResponse
from rclpy.action import ActionServer

class MockControllerIntegrationNode(Node):
    def __init__(self, namespace='colav_gateway', node_name='controller_integration_node'):
        super().__init__(node_name=node_name, namespace=namespace)
        self.execute_mission_action_server = ActionServer(
            self,
            MissionExecutor,
            f"{namespace}/{node_name}",
            self.execute
        )