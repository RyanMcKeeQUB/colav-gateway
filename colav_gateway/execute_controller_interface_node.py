import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from utils.config_extractor_utils import extract_endpoint, EndpointEnum
import threading
import sys
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_prefix
import os

# package_name = 'colav_gateway'
# install_path = get_package_prefix(package_name=package_name)
# site_packages_path = os.path.join(install_path, "lib", "python3.10", "site-packages")

from colav_gateway.scripts.controller_interface_node import ControllerInterfaceNode

logger = get_logger("controller_interface_node")


def main(args=None):

    try:
        rclpy.init(args=args)
        controller_interface_node = ControllerInterfaceNode()

        executor = MultiThreadedExecutor()
        executor.add_node(node = controller_interface_node) 
        executor_thread = threading.Thread(target=executor.spin, daemon=True) # set daemon to true in future to make background worker
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