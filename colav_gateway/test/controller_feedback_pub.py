import rclpy
from rclpy.node import Node
from colav_interfaces.msg import ControllerFeedback
from colav_interfaces.msg import CmdVelYaw, ControlMode, ControlStatus
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

class ControllerFeedbackPublisher(Node):
    def __init__(self):
        super().__init__('controller_feedback_publisher')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.publisher_ = self.create_publisher(ControllerFeedback, '/controller_feedback', qos_profile)
        self.timer = self.create_timer(1.0, self.publish_feedback)

    def publish_feedback(self):
        msg = ControllerFeedback()
        msg.mission_tag = ''
        msg.agent_tag = ''
        msg.cmd_vel_yaw = CmdVelYaw(velocity=0.0, yaw_rate=0.0)
        msg.ctrl_mode = ControlMode(control_mode=0)
        msg.ctrl_status = ControlStatus(status=0, message='')
        msg.timestamp = ''
        msg.timestep = ''
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing feedback')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerFeedbackPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
