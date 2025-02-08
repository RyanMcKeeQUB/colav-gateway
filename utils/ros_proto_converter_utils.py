from controllerFeedback_pb2 import ControllerFeedback as ProtoControllerFeedback
from colav_interfaces.msg import ControllerFeedback as ROSControllerFeedback

class ROSTOProtoUtils:

    @staticmethod
    def parse_controller_feedback(msg:ROSControllerFeedback) -> ProtoControllerFeedback:
        """Parse controller feedback to ROS message"""
        try: 
            proto_ctrl_msg = ProtoControllerFeedback()
            proto_ctrl_msg.mission_tag = msg.mission_tag
            proto_ctrl_msg.agent_tag = msg.agent_tag
            proto_ctrl_msg.ctrl_mode = msg.ctrl_mode.control_mode
            proto_ctrl_msg.ctrl_status.status = msg.ctrl_status.status
            proto_ctrl_msg.ctrl_status.message = msg.ctrl_status.message
            proto_ctrl_msg.ctrl_cmd.velocity = msg.cmd_vel_yaw.velocity
            proto_ctrl_msg.ctrl_cmd.yaw_rate = msg.cmd_vel_yaw.yaw_rate
            proto_ctrl_msg.timestamp = msg.timestamp
            proto_ctrl_msg.timestep = msg.timestep

            return proto_ctrl_msg
        except Exception as e:
            raise ValueError(f"Failed to parse exception: {e}") from e
        

def main(args = None):
    mock_ros_msg = ROSControllerFeedback()
    mock_ros_msg.mission_tag = "MISSION"
    mock_ros_msg.agent_tag = "EF12_WORKBOAT"
    mock_ros_msg.cmd_vel_yaw.velocity = float(15)
    mock_ros_msg.cmd_vel_yaw.yaw_rate = float(0.2)
    mock_ros_msg.ctrl_mode.control_mode = 3
    mock_ros_msg.ctrl_status.status = 1
    mock_ros_msg.ctrl_status.message = "Mission is active, no error!"

    mock_ros_msg.timestamp = "23/10/2025 12:23:44"
    mock_ros_msg.timestep = "000000000010323100"

    ros_to_proto_utils = ROSTOProtoUtils()
    protobuf_msg = ros_to_proto_utils.parse_controller_feedback(msg = mock_ros_msg)

    print (protobuf_msg)


if __name__ == '__main__':
    main()