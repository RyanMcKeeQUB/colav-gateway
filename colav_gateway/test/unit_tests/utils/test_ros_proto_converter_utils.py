import unittest
from utils.ros_proto_converter_utils import ROSTOProtoUtils
from colav_interfaces.msg import ControllerFeedback as ROSControllerFeedback
from controllerFeedback_pb2 import ControllerFeedback as ProtoControllerFeedback

class TestROSControllerFeedback(unittest.TestCase):

    def setUp(self):
        """initiaialisation for the test"""
        self._mock_ctrl_feedback = ROSControllerFeedback()
        self._mock_ctrl_feedback.mission_tag = "MISSION"
        self._mock_ctrl_feedback.agent_tag = "EF12_WORKBOAT"
        self._mock_ctrl_feedback.cmd_vel_yaw.velocity = float(15)
        self._mock_ctrl_feedback.cmd_vel_yaw.yaw_rate = float(0.2)
        self._mock_ctrl_feedback.ctrl_mode.control_mode = 3
        self._mock_ctrl_feedback.ctrl_status.status = 1
        self._mock_ctrl_feedback.ctrl_status.message = "Mission is active, no error!"

        self._mock_ctrl_feedback.timestamp = "23/10/2025 12:23:44"
        self._mock_ctrl_feedback.timestep = "000000000010323100"

        self.ros_to_proto_utils = ROSTOProtoUtils()

        return super().setUp()
    
    def test_parse_controller_feedback(self):
        """parse controller feedback"""
        self.ros_ctrl_msg = self.ros_to_proto_utils.parse_controller_feedback(msg=self._mock_ctrl_feedback)
        
        self.assertEqual(self._mock_ctrl_feedback.mission_tag, self.ros_ctrl_msg.mission_tag)
        self.assertEqual(self._mock_ctrl_feedback.agent_tag, self.ros_ctrl_msg.agent_tag)
        self.assertEqual(self._mock_ctrl_feedback.ctrl_cmd.velocity, self.ros_ctrl_msg.cmd_vel_yaw.velocity)
        self.assertEqual(self._mock_ctrl_feedback.ctrl_cmd.yaw_rate, self.ros_ctrl_msg.cmd_vel_yaw.yaw_rate)
        self.assertEqual(self._mock_ctrl_feedback.ctrl_mode.yaw_rate, self.ros_ctrl_msg.ctrl_mode.control_mode)
        self.assertEqual(self._mock_ctrl_feedback.ctrl_status.status, self.ros_ctrl_msg.ctrl_status.status)
        self.assertEqual(self._mock_ctrl_feedback.ctrl_status.message, self.ros_ctrl_msg.ctrl_status.message)
        self.assertEqual(self.mock_ctrl_feedback, self.ros_ctrl_msg.timestamp)
        self.assertEqual(self._mock_ctrl_feedback, self.ros_ctrl_msg.timestep)
        # self.assertEqual(self._mock_ctrl_feedback.ctrl_mode.control_mode, )
        # self.assertEqual(self._mock_ctrl_feedback.ctrl_status.status, )
        # self.assertEqual(self._mock_ctrl_feedback.ctrl_status.message, )
        # self.assertEqual(self._mock_ctrl_feedback.timestamp, )
        # self.assertEqual(self._mock_ctrl_feedback.timestep, )

if __name__ == "__main__":
    unittest.main()