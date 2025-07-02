import rclpy
from rclpy.node import Node
from sensor_msgs.msg import IMU
from geometry_msgs.msg import PoseArray, Point, Quaternion, Pose
from .parameters import PARAMETERS

class Distance_Tracker(Node):

    def __init__(self):
        super().__init__('Distance_Tracker')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        self._subscription = self.create_subscription(IMU, self.get_parameter("imu_topic").value, self._on_imu, 10)
        self._publisher = self.create_publisher(DistanceTotal, self.get_parameter("distance_total_topic").value, 10)
        # self.timer = self.create_timer(timer_period, self._timer_callback)

    def _on_imu(self, msg):
        self.get_logger().info('Received: "%s"' % type(msg))
        # add code to integrate IMU data to calculate distance
        # can use this to output the position as well if needed
        # publish to DistanceTotal topic


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Planner()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()