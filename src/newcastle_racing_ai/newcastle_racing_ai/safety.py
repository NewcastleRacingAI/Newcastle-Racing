import rclpy
from rclpy.node import Node
from eufs_msgs.msg import  CanState
from newcastle_racing_ai_msgs.msg import EBS
from .parameters import PARAMETERS

class Safety(Node):

    def __init__(self):
        super().__init__('Safety')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        self._subscription = self.create_subscription(CanState, self.get_parameter("can_state_topic").value, self._on_state, 10)
        self._publisher = self.create_publisher(EBS, self.get_parameter("ebs_topic").value, 10)
        self._ebs_active = False
        self._timer = self.create_timer(0.1, self._publish_ebs)  # 10 Hz


    def _on_state(self, msg):
        self.get_logger().info('Received: "%s"' % type(msg))
        if msg.as_state == AS_EMERGENCY_BRAKE or msg.ami_state == AMI_ADS_EBS:
            self.ebs_active = True

    
    def _publish_ebs(self):
        if self.ebs_active:
            self._publisher.publish(self.ebs_active)
        else:
            pass



def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Planner()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()