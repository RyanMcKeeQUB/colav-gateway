import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.logging import get_logger
from colav_gateway.utils.config_extractor_utils import extract_endpoint, EndpointEnum
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
    (mission_request_host, mission_request_port) = extract_endpoint(endpoint_config=endpoint_data, key=EndpointEnum.MISSION_REQUEST)
    (mission_response_host, mission_response_port) = extract_endpoint(endpoint_config=endpoint_data, key=EndpointEnum.MISSION_RESPONSE)

    (agent_config_host, agent_config_port) = extract_endpoint(endpoint_config=endpoint_data, key=EndpointEnum.AGENT_CONFIG)
    (obstacles_config_host, obstacles_config_port) = extract_endpoint(endpoint_config=endpoint_data, key=EndpointEnum.OBSTACLES_CONFIG)

    (controller_feedback_host, controller_feedback_port) = extract_endpoint(endpoint_config=endpoint_data, key=EndpointEnum.CONTROLLER_FEEDBACK)

    if (not mission_request_host or not mission_request_port 
        or not mission_response_host or not mission_response_port
        or not agent_config_host or not agent_config_port
        or not obstacles_config_host or not obstacles_config_port
        or not controller_feedback_host or not controller_feedback_port):
        raise ValueError("Extracted endpoint data is invalid.")

    return LaunchDescription([
        DeclareLaunchArgument(
            'mission_request_host',
            default_value=str(mission_request_host),  # Convert tuple to string
            description="host of mission request"
        ),
        DeclareLaunchArgument(
            'mission_request_port',
            default_value=str(mission_request_port),  # Convert tuple to string
            description="port of mission response"
        ),
        DeclareLaunchArgument(
            'mission_response_host',
            default_value=str(mission_response_host),  # Convert tuple to string
            description="host of mission response"
        ),
        DeclareLaunchArgument(
            'mission_response_port',
            default_value=str(mission_response_port),  # Convert tuple to string
            description="port of mission response"
        ),
        DeclareLaunchArgument(
            'agent_config_host',
            default_value=str(agent_config_host),  # Convert tuple to string
            description="host of agent config updates"
        ),
        DeclareLaunchArgument(
            'agent_config_port',
            default_value=str(agent_config_port),
            description="port of agent config updates"
        ),
        DeclareLaunchArgument(
            'obstacles_config_host',
            default_value=str(obstacles_config_host),  # Convert tuple to string
            description="host of obstacle config updates"
        ),
        DeclareLaunchArgument(
            'obstacles_config_port',
            default_value=str(obstacles_config_port),  # Convert tuple to string
            description="port of obstacle config updates"
        ),
        DeclareLaunchArgument(
            'controller_feedback_host',
            default_value=str(controller_feedback_host),  # Convert tuple to string
            description="host of controller feedback"
        ),
        DeclareLaunchArgument(
            'controller_feedback_port',
            default_value=str(controller_feedback_port),  # Convert tuple to string
            description="port of controller feedback"
        ),
        Node(
            package=package_name,
            executable='controller_interface_node',
            name='controller_interface',
            output='screen',
            parameters=[{
                'agent_config_host': str(agent_config_host),
                'agent_config_port': int(agent_config_port),
                'obstacles_config_host': str(obstacles_config_host),  
                'obstacles_config_port': int(obstacles_config_port),
            }],
        ),
        Node(
            package=package_name,
            executable='mission_interface_node',
            name='mission_interface',
            output='screen',
            parameters=[{
                'mission_request_host': str(mission_request_host),  
                'mission_request_port': int(mission_request_port),
                'mission_response_host': str(mission_response_host), 
                'mission_response_port': int(mission_response_port),
                'controller_feedback_host': str(controller_feedback_host),
                'controller_feedback_port': int(controller_feedback_port)
            }],
        ),
    ])

def main(args=None):
    generate_launch_description()

if __name__ == '__main__':
    main()
