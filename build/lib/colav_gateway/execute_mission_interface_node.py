import rclpy

import sys
from rclpy.logging import get_logger
import threading
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_prefix
import os

# package_name = 'colav_gateway'
# install_path = get_package_prefix(package_name=package_name)
# site_packages_path = os.path.join(install_path, "lib", "python3.10", "site-packages")

from colav_gateway.scripts.mission_interface_node import MissionInterfaceNode

logger = get_logger("execute_mission_interface_node")

def executor_thread_fn(executor):
    """ Runs the executor in a separate thread. """
    try:
        executor.spin()
    except Exception as e:
        logger.error(f"Executor thread exception: {e}")
    finally:
        logger.info("Shutting down executor...")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    mission_interface_node = None
    executor = None
    executor_thread = None

    try:
        mission_interface_node = MissionInterfaceNode()
        executor = MultiThreadedExecutor()
        executor.add_node(mission_interface_node)

        executor_thread = threading.Thread(target=executor_thread_fn, args=(executor,), daemon=True)
        executor_thread.start()

        while rclpy.ok():
            # This keeps the main thread alive while monitoring for exceptions
            executor_thread.join(timeout=1.0)
            if not executor_thread.is_alive():
                logger.error("Executor thread unexpectedly stopped!")
                break

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received. Shutting down...")
    except Exception as e:
        logger.error(f"Main thread exception: {e}")
    finally:
        if mission_interface_node:
            mission_interface_node.destroy_node()
            logger.info("Node destroyed.")
        if executor:
            executor.shutdown()  # Ensure executor stops
        rclpy.shutdown()  # Properly shut down ROS
        logger.info("ROS shutdown complete.")

if __name__ == "__main__":
    main()
