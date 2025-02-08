import rclpy
from scripts.mission_interface_node import MissionInterfaceNode
import sys
from rclpy.logging import get_logger

logger = get_logger("execute_mission_interface_node")

def main(args=None):
    try:
        rclpy.init(args=args)
        node = MissionInterfaceNode()
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt occured closing controller_interface_node")
    except Exception as e:
        logger.error(f"{e}")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == "__main__":
    main()