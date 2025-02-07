import rclpy
from utils.config_extractor_utils import extract_endpoint, EndpointEnum
from scripts.controller_interface_node import ControllerInterfaceNode
import sys
from rclpy.logging import get_logger
import yaml

logger = get_logger("execute_controller_interface_node")

def parse_yaml(yml_path: str):
    """parses a yml file"""
    try: 
        with open(yml_path, "r") as yml_file:
            yml_data = yaml.safe_load(yml_file)
            
        return yml_data
    except Exception as e:
        raise e

def main(args=None):
    try:
        rclpy.init(args=args)
        try:
            endpoint_config = parse_yaml(rclpy.get_param('endpoint_config_path'))
        except Exception as e:
            logger.error(f'{e}')
            sys.exit(1)
            rclpy.shutdown()

        node = ControllerInterfaceNode(
            agent_config_address=extract_endpoint(endpoint_config=endpoint_config, key=EndpointEnum.AGENT_CONFIG),
            obstacles_config_address=extract_endpoint(endpoint_config=endpoint_config, key=EndpointEnum.OBSTACLES_CONFIG)
        )
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