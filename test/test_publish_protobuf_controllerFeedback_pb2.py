import controllerFeedback_pb2
import socket

control_feedback = controllerFeedback_pb2.ControllerFeedback()

control_feedback.mission_tag = "mock_mission"
control_feedback.agent_tag = "EF12_WORKBOAT"
control_feedback.ctrl_mode = controllerFeedback_pb2.ControllerFeedback.CTRLMode.CRUISE
control_feedback.ctrl_status = (
    controllerFeedback_pb2.ControllerFeedback.CTRLStatus.ACTIVE
)
control_feedback.ctrl_cmd.velocity = float(25.0)
control_feedback.ctrl_cmd.yaw_rate = float(0.2)
control_feedback.timestamp = "01/02/2025 13:23:12"
control_feedback.timestep = "000000000000000001"

serialised_control_feedback = control_feedback.SerializeToString()
print(f"Serialised msg: {serialised_control_feedback}")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(serialised_control_feedback, ("0.0.0.0", 7300))
