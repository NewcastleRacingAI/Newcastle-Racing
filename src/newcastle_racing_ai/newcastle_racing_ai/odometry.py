import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, PoseWithCovariance, Twist, TwistWithCovariance
import numpy as np
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from .parameters import PARAMETERS

class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        imu_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, self.get_parameter("imu_topic").get_parameter_value().string_value, self.imu_callback, imu_qos)
        self.odom_publisher = self.create_publisher(Odometry, self.get_parameter("odom_topic").get_parameter_value().string_value, 10)
        self.timer = self.create_timer(0.01, self.publish_odometry)
        self.prev_time = None
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.imu = Imu()

    def imu_callback(self, msg):
        self.imu = msg


    def publish_odometry(self):
        imu = self.imu
        self.get_logger().info(f"Received IMU data: {imu.header.stamp.sec}.{imu.header.stamp.nanosec} - Acceleration: {imu.linear_acceleration.x}, {imu.linear_acceleration.y}, {imu.linear_acceleration.z}")
        current_time = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        self.prev_time = current_time

        acc = np.array([
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z
        ])

        self.velocity += acc * dt
        self.position += self.velocity * dt

        odom_msg = Odometry()
        odom_msg.header.stamp = imu.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position = Point(x=self.position[0], y=self.position[1], z=self.position[2])
        odom_msg.pose.pose.orientation = imu.orientation
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = self.velocity[2]
        odom_msg.twist.twist.angular = imu.angular_velocity

        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
