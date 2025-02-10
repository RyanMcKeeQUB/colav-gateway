import launch.launch_description_sources
import rclpy 
import unittest
import launch
import launch_testing
import launch_ros.actions
from rclpy.node import Node
from scripts.controller_interface_node import ControllerInterfaceNode
import pytest
import subprocess


class TestIntegrationColavGateway(unittest.TestCase):
    
    @pytest.fixture(autouse=True)
    def setup_and_teardown(self):
            """Fixture to initialize and shutdown ROS 2 nodes."""
            self.controller_interface_node_process = subprocess.Popen(
                 ['ros2', 'run', 'colav_gateway', 'colav_gateway', 'controller_interface_node']
            )

    @classmethod
    def tearDownClass(cls):
        cls.process.shutdown()

    def test_node_runs(self):
        self.assertTrue(self.process.is_alive())


if __name__ == "__main__":
    unittest.main()