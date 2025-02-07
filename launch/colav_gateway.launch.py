import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.logging import get_logger
from utils.config_extractor_utils import extract_endpoint, EndpointEnum
import yaml
import os

logger = get_logger("colav_gateway.launch.py")
package_name = 'colav_gateway'

def parse_yaml(yml_path: str):
    """Parses a YAML file"""
    if not os.path.exists(yml_path):
        raise FileNotFoundError(f"YAML file not found: {yml_path}")

    try:
        with open(yml_path, "r") as yml_file:
            return yaml.safe_load(yml_file)
    except yaml.YAMLError as e:
        raise RuntimeError(f"Error parsing YAML: {e}")

def generate_launch_description():
    try:
        yaml_path = os.path.join(
            get_package_share_directory(package_name),
            'config',
            'endpoint_config.yml'
        )

        endpoint_data = parse_yaml(yaml_path)
    except Exception as e:
        raise RuntimeError(f"Failed to parse endpoint config: {e}")

    # Extract endpoints (returns tuples)
    (agent_config_host, agent_config_port) = extract_endpoint(endpoint_config=endpoint_data, key=EndpointEnum.AGENT_CONFIG)
    (obstacles_config_host, obstacles_config_port) = extract_endpoint(endpoint_config=endpoint_data, key=EndpointEnum.OBSTACLES_CONFIG)

    if (not agent_config_host or not agent_config_port
        or not obstacles_config_host or not obstacles_config_port):
        raise ValueError("Extracted endpoint data is invalid.")

    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_config_host',
            default_value=agent_config_host,  # Convert tuple to string
            description="host of agent config updates"
        ),
        DeclareLaunchArgument(
            'agent_config_port',
            default_value=agent_config_port,
            description="port of agent config updates"
        ),
        DeclareLaunchArgument(
            'obstacles_config_host',
            default_value=obstacles_config_host,  # Convert tuple to string
            description="host of obstacle config updates"
        ),
        DeclareLaunchArgument(
            'obstacles_config_port',
            default_value=obstacles_config_port,  # Convert tuple to string
            description="port of obstacle config updates"
        ),
        Node(
            package=package_name,
            executable='controller_interface_node',
            name='controller_interface',
            output='screen',
            parameters=[{
                'agent_config_host': str(agent_config_host),  # Use list instead of tuple
                'agent_config_port': int(agent_config_port),
                'obstacles_config_host': str(obstacles_config_host),  # Use list instead of tuple
                'obstacles_config_port': int(obstacles_config_port),
            }],
        )
    ])

def main(args=None):
    generate_launch_description()

if __name__ == '__main__':
    main()
