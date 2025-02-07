import rclpy
from scripts.mission_interface_node import MissionInterfaceNode

def main(args=None):
    rclpy.init()
    node = MissionInterfaceNode(
        mission_request_address=('0.0.0.0', 7000),
        mission_response_address=('0.0.0.0', 7001),
        controller_feedback_address=('0.0.0.0', 7300)
    )

    try: 
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        rclpy.logging.get_logger().info('Keyboard interupt, shutting down ros node!')
    except Exception as e:
        rclpy.logging.get_logger().error(f'Error occured {e}')
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()