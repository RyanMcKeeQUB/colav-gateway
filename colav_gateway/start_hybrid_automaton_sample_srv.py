#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from colav_interfaces.srv import StartHybridAutomaton

class StartHybridAutomatonService(Node):
    def __init__(self):
        super().__init__("start_hybrid_automaton_service")
        self.srv = self.create_service(StartHybridAutomaton, "/start_hybrid_automaton", self.handle_service_request)
        self.get_logger().info("StartHybridAutomaton Service is ready.")

    def handle_service_request(self, request, response):
        """Handles incoming service requests."""
        mission_request = request.mission_request
        self.get_logger().info(f"Received mission request: {mission_request.mission_tag}")

        # Simulate some processing logic (replace this with actual logic)
        # if mission_request.mission_tag:
        response.success = True
        response.message = f"Hybrid Automaton started successfully for mission: {mission_request.mission_tag}"
        # else:
        #     response.success = False
        #     response.message = "Mission request is invalid."

        self.get_logger().info(f"Response: success={response.success}, message={response.message}")
        return response

def main():
    rclpy.init()
    node = StartHybridAutomatonService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
