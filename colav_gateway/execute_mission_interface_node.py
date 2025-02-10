import rclpy
from scripts.mission_interface_node import MissionInterfaceNode
import sys
from rclpy.logging import get_logger
import threading

logger = get_logger("execute_mission_interface_node")

def start_udp_listener(node, error_event):
    try:
        node.listen_for_mission_request()
    except Exception as e:
        # Set the error event and store the exception to raise it in the main thread.
        error_event.set()
        error_event.exception = e
        # Shutdown rclpy so that rclpy.spin() exits in the main thread, if not already shut down.
        if rclpy.ok():
            rclpy.shutdown()

def main(args=None):
    node = None  # Initialize node to None to avoid UnboundLocalError
    error_event = threading.Event()  # Event to signal an error
    try:
        rclpy.init(args=args)
        node = MissionInterfaceNode()

        listener_thread = threading.Thread(target=start_udp_listener, args=(node, error_event))
        listener_thread.start()

        # rclpy.spin() will block until rclpy.shutdown() is called.
        rclpy.spin(node=node)
        listener_thread.join()

        # Check if there was an error in the thread after spin returns.
        if error_event.is_set():
            raise error_event.exception  # Reraise the exception that occurred in the thread.
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt occurred, closing controller_interface_node")
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        if node:  # Ensure the node is destroyed only if it was successfully created.
            node.destroy_node()
        # Only call shutdown if the ROS context is still active.
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)

if __name__ == "__main__":
    main()
