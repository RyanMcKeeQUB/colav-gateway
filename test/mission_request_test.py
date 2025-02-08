import pytest
import rclpy
from rclpy.node import Node
from colav_interfaces.msg import Mission
import socket

from proto_gen.missionRequest_pb2 import MissionRequest
class TestMissionRequestTopic(Node):
    def __init__(self):
        super().__init__('test_colav_gateway_mission_request_node')
        mission_request_topic = "colav/mission_request"
        self.mission_request_topic = self.create_subscription(
            Mission,
            mission_request_topic,
            self.listener_callback,  # Fix callback name
            10
        )   
        self.received_msg = None  # Initialize received_msg

    def listener_callback(self, msg):
        # Store the received message data
        self.received_msg = msg.data

@pytest.fixture
def protobuf_msg():
    mock_mission_request = MissionRequest()
    mock_mission_request.tag = "MOCK_MISSION_REQUEST"
    mock_mission_request.mission_start_timestamp = "2025-01-31 15:30:45"

    mock_mission_request.vessel.tag = "EF-12 WORKBOAT"
    mock_mission_request.vessel.type = MissionRequest.Vessel.VesselType.HYDROFOIL

    mock_mission_request.vessel.vessel_constraints.max_acceleration = 3.0 # m**2/s
    mock_mission_request.vessel.vessel_constraints.max_deceleration = 2.0 # m**2/s
    mock_mission_request.vessel.vessel_constraints.max_velocity = 15.956 # m/s
    mock_mission_request.vessel.vessel_constraints.min_velocity = 7.717 # m/s
    mock_mission_request.vessel.vessel_constraints.max_yaw_rate = 0.2 # rad/s

    mock_mission_request.vessel.vessel_geometry.safety_threshold = 5 # safety radius in meters

    x = 1
    y = 1
    z = 0
    # Define the polyshape of the vessel
    for i in range(1, 5): 
        point = mock_mission_request.vessel.vessel_geometry.polyshape_points.add()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        x += 1
        y += 1

    mock_mission_request.mission_init_position.x = float(1.0)  # Initial mission position
    mock_mission_request.mission_init_position.y = float(1.0)
    mock_mission_request.mission_init_position.z = float(0.0)

    mock_mission_request.mission_goal_position.x = float(1.0)  # Mission goal position
    mock_mission_request.mission_goal_position.z = float(1.0)
    mock_mission_request.mission_goal_position.y = float(1.0)

    yield mock_mission_request

@pytest.fixture
def test_mission_request_ros_node():
    rclpy.init()
    node = TestMissionRequestTopic()
    yield node
    rclpy.shutdown()

def test_mission_request_publish_subscribe(protobuf_msg, test_mission_request_ros_node):
    # Ensure message is serialized properly
    serialized_protobuf_msg = protobuf_msg.SerializeToString()

    # Create a UDP socket for sending the message (used here for simulating external communication)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(serialized_protobuf_msg, ("0.0.0.0", 9999))

    # Allow time for the message to be received and callback to trigger
    rclpy.spin_once(test_mission_request_ros_node)

    # Check if the message was received
    assert test_mission_request_ros_node.received_msg is not None, "No message was received."
    assert test_mission_request_ros_node.received_msg == protobuf_msg.SerializeToString(), "The received message does not match the expected protobuf message."
    
    print('Test completed successfully')

if __name__ == "__main__":
    # Run pytest programmatically (optional)
    pytest.main()