import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Point, Quaternion, Pose
from eufs_msgs.msg import ConeArrayWithCovariance
from .parameters import PARAMETERS

class Planner(Node):

    def __init__(self):
        super().__init__('Planner')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        self._subscription = self.create_subscription(ConeArrayWithCovariance, self.get_parameter("cones_topic").value, self._on_cones, 10)
        self._publisher = self.create_publisher(PoseArray, self.get_parameter("path_topic").value, 10)
        # self.timer = self.create_timer(timer_period, self._timer_callback)

    def _on_cones(self, msg):
        self.get_logger().info('Received: "%s"' % type(msg))

    def _timer_callback(self):
        msg = PoseArray(poses=[
            Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0))
        ])
        self._publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Planner()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()