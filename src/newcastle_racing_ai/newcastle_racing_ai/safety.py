import rclpy
from rclpy.node import Node
from eufs_msgs.msg import  CanState
from newcastle_racing_ai_msgs.msg import EBS, MissionState


from .parameters import PARAMETERS

class Safety(Node):

    def __init__(self):
        super().__init__('Safety')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        self._subscription = (MissionState, self.get_parameter("mission_state_topic").value, self._on_mission_state, 10)
        self._subscription = self.create_subscription(CanState, self.get_parameter("can_state_topic").value, self._on_can_state, 10)
        self._publisher = self.create_publisher(EBS, self.get_parameter("ebs_topic").value, 10)
        self._ebs_active = EBS()
        self._ebs_active.ebs = False  # Initialize EBS state
        self._timer = self.create_timer(0.1, self._publish_ebs)  # 10 Hz


    def _on_can_state(self, msg):
        # for message directly from the CAN bus
        if msg.as_state == CanState.AS_EMERGENCY_BRAKE or msg.ami_state == CanState.AMI_ADS_EBS:
            self.get_logger().info('Received from CAN: as_state=%s, ami_state=%s' % (msg.as_state, msg.ami_state))
            self._ebs_active = True

    def _on_mission_state(self, msg):
        # for messages from mission control
        if msg.mission_state == MissionState.AS_EMERGENCY_BRAKE:
            self.get_logger().info('Received from mission control: "%s"' % msg.mission_state)
            self._ebs_active = True

    
    def _publish_ebs(self):
        if self._ebs_active:
            # need to add functionality to send this to the VCU as well. 
            self._publisher.publish(self._ebs_active)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Safety()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()