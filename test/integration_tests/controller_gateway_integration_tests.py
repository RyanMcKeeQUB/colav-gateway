import launch.launch_description_sources
import rclpy 
import unittest
import launch
import launch_testing
import launch_ros.actions
from rclpy.node import Node

class TestIntegrationColavGateway(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        """Start the node before running tests."""
        cls.launch_service = launch.LaunchService()
        cls.process = launch_ros.actions.Node(
            package="my_package",
            executable="my_node",
            name="my_test_node"
        )
        cls.launch_service.include_launch_description(
            launch.LaunchDescription([cls.process])
        )
        cls.launch_service.run()

    @classmethod
    def tearDownClass(cls):
        cls.process.shutdown()

    def test_node_runs(self):
        self.assertTrue(self.process.is_alive())


if __name__ == "__main__":
    unittest.main()