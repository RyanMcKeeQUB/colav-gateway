import socket
import rclpy
from rclpy.node import Node
from mission_request.mission_request import MissionRequestNode
import argparse
import json
import os
import rclpy.logging

logger = rclpy.logging.get_logger("colav_gateway_logger")
workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class ColavGateway(Node):
    def __init__(
        self,
        config: json,
    ):
        super().__init__("colav_gateway")
        self.get_logger().info("colav_gateway node started!")
        self.__init_colav__(config=config)

    def __init_colav__(self, config: json):
        # Initiates colav gateway which begins listening for mission requests.
        self.get_logger().info("Initiating COLAV.")
        mission_request_node = MissionRequestNode(
            req_address=(
                config["endpoint_config"]["mission_request"]["host"],
                config["endpoint_config"]["mission_request"]["port"],
            ),
            res_address=(
                config["endpoint_config"]["mission_response"]["host"],
                config["endpoint_config"]["mission_response"]["port"],
            ),
            workspace_dir=workspace_dir,
        )
        while rclpy.ok:
            rclpy.spin(mission_request_node)
            mission_request_node.destroy_node()
        return


def main():
    logger.info("Parsing colav_gateway config.json.")
    parser = argparse.ArgumentParser(
        description="Script that takes colav_gateway_config.json as arg"
    )
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        help="Path to the config JSON file for colav_gateway.",
        required=True,
    )
    args = parser.parse_args()

    try:
        with open(args.config, "r") as colav_gateway_config_file:
            colav_gateway_config = json.load(colav_gateway_config_file)
            logger.info("colav gateway config.json successfully read in.")
    except Exception as e:
        logger.error("Invalid colav_gateway_config.json passed as colav_gateway_arg.")
        exit()

    rclpy.init(args=None)
    colav_gateway = ColavGateway(config=colav_gateway_config)
    rclpy.spin(colav_gateway)
    colav_gateway.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    logger.info("Starting colav_gateway application.")
    main()
