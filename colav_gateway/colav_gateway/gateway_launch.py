import rclpy
import sys
import os
import argparse
import yaml
import threading
from rclpy.node import Node
from rclpy.logging import get_logger

# from ament_index_python.packages import get_package_prefix
# import os

# package_name = 'colav_gateway'
# install_path = get_package_prefix(package_name=package_name)
# site_packages_path = os.path.join(install_path, "lib", "python3.10", "site-packages")

# if site_packages_path in sys.path:
#     sys.path.remove(site_packages_path)  # Remove if it already exists
# sys.path.insert(0, site_packages_path)  # Insert at the highest priority

from colav_gateway.utils.config_extractor_utils import extract_endpoint, EndpointEnum

# Add custom package path
from colav_gateway.scripts.mission_interface_node import MissionInterfaceNode
from colav_gateway.scripts.controller_interface_node import ControllerInterfaceNode

logger = get_logger("gateway_launch")

def spin_node(node):
    """Spin a ROS 2 node in a separate thread"""
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info('Node shutting down')
    except Exception as e:
        logger.error(f'Exception occurred: {e}')
    finally:
        node.destroy_node()

def create_controller_interface_node(endpoint_config):
    return ControllerInterfaceNode(
        agent_config_address=extract_endpoint(endpoint_config, EndpointEnum.AGENT_CONFIG),
        obstacles_config_address=extract_endpoint(endpoint_config, EndpointEnum.OBSTACLES_CONFIG)
    )

def create_mission_interface_node(endpoint_config):
    return MissionInterfaceNode(
        mission_request_address=extract_endpoint(endpoint_config, EndpointEnum.MISSION_REQUEST), 
        mission_response_address=extract_endpoint(endpoint_config, EndpointEnum.MISSION_RESPONSE),
        controller_feedback_address=extract_endpoint(endpoint_config, EndpointEnum.CONTROLLER_FEEDBACK)
    )

def main():
    parser = argparse.ArgumentParser(description="COLAV Gateway Configuration")
    parser.add_argument("-e", "--endpoint_config", type=str, required=True, help="Path to YML endpoint config file")
    args = parser.parse_args()

    # Load endpoint configuration
    try:
        with open(args.endpoint_config, "r") as endpoint_config_file:
            endpoint_config = yaml.safe_load(endpoint_config_file)
    except Exception as e:
        logger.error(f"Failed to load endpoint config: {e}")
        sys.exit(1)

    # Initialize ROS 2
    rclpy.init(args=None)

    # Instantiate nodes lazily
    nodes = {
        "controller_interface_node": lambda: create_controller_interface_node(endpoint_config),
        "mission_interface_node": lambda: create_mission_interface_node(endpoint_config)
    }

    # Thread management
    threads = []
    stop_event = threading.Event()

    for name, node_creator in nodes.items():
        # Instantiate the node only when the thread starts
        thread = threading.Thread(target=spin_node, args=(node_creator(),), daemon=True)
        threads.append(thread)
        thread.start()
        logger.info(f"Started {name} in a separate thread.")

    try:
        for thread in threads:
            thread.join()  # Keep main script running
    except KeyboardInterrupt:
        logger.info("Shutting down nodes...")
        stop_event.set()
        for node_creator in nodes.values():
            node_creator().destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
