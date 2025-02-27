import rclpy
from rclpy.node import Node
from colav_interfaces.msg import ControllerFeedback
from colav_interfaces.msg import CmdVelYaw, ControlMode, ControlStatus
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

class ControllerFeedbackSubscriber(Node):
    def __init__(self):
        super().__init__('controller_feedback_subscriber')
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Subscriber to the '/controller_feedback' topic
        self.subscription = self.create_subscription(
            ControllerFeedback,
            '/controller_feedback',
            self.feedback_callback,
            qos_profile
        )
        self.subscription  # Prevent unused variable warning

    def feedback_callback(self, msg):
        self.get_logger().info(f'Received feedback: {msg}')
        # You can process the ControllerFeedback message here
        # For example, log the cmd_vel_yaw:
        self.get_logger().info(f'Command Velocity: {msg.cmd_vel_yaw.velocity}, Yaw Rate: {msg.cmd_vel_yaw.yaw_rate}')
        self.get_logger().info(f'Control Mode: {msg.ctrl_mode.control_mode}')
        self.get_logger().info(f'Control Status: {msg.ctrl_status.status}, Message: {msg.ctrl_status.message}')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerFeedbackSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
