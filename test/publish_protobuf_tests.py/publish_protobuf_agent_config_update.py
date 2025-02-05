from agentUpdate_pb2 import AgentUpdate as ProtobufAgentCOnfigUpdate
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

agent_config_update = ProtobufAgentCOnfigUpdate()
# METADATA
agent_config_update.mission_tag = "MOCK_MISSION"
agent_config_update.agent_tag = "EF12_WORKBOAT"
# Position
agent_config_update.state.pose.position.x = float(20)
agent_config_update.state.pose.position.y = float(20)
agent_config_update.state.pose.position.z = float(20)
# Orientation
agent_config_update.state.pose.orientation.x = float(20)
agent_config_update.state.pose.orientation.y = float(20)
agent_config_update.state.pose.orientation.z = float(20)
agent_config_update.state.pose.orientation.w = float(20)

agent_config_update.state.velocity = float(15.0)
agent_config_update.state.yaw_rate = float(0.2)
agent_config_update.state.acceleration = float(5.0)

agent_config_update.timestamp = "12/01/2025 12:02:24"
agent_config_update.timestep = "000000001"

data = agent_config_update.SerializeToString()
sock.sendto(data, ("0.0.0.0", 7100))