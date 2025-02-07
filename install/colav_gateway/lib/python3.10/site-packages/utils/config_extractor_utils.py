from enum import Enum

class EndpointEnum(Enum):
    MISSION_REQUEST = 'mission_request'
    MISSION_RESPONSE = 'mission_response'
    CONTROLLER_FEEDBACK = 'controller_feedback'
    AGENT_CONFIG = 'agent_config'
    OBSTACLES_CONFIG = 'obstacles_config'

@staticmethod
def extract_endpoint(endpoint_config: dict, key: EndpointEnum) -> tuple[str, int]:
    try:
        return (endpoint_config[key.value]['host'], endpoint_config[key.value]['port'])
    except KeyError as e:
        raise ValueError(f"Missing key '{key.value}' in config") from e
    except Exception as e:
        raise ValueError(f"Error parsing {key.value} endpoint from config") from e

  