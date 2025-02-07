import rclpy
from scripts.controller_interface_node import ControllerInterfaceNode

def main(args=None):
    rclpy.init()
    node = ControllerInterfaceNode(
        ('0.0.0.0', 7100),
        ('0.0.0.0', 7200)
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