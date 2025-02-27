from colav_interfaces.msg import MissionRequest

def validate_mission_request(self, mission_request: MissionRequest) -> MissionRequest:
    """Validates the mission request data before sending to hybrid automaton."""
    # TODO: Add mission request validation
    return mission_request