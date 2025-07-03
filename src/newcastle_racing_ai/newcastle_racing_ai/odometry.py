import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, PoseWithCovariance, Twist, TwistWithCovariance
import numpy as np
import tf_transformations
from builtin_interfaces.msg import Time
from .parameters import PARAMETERS

class Odometry(Node):

    def __init__(self):
        super().__init__('odometry')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        self.subscription = self.create_subscription(Imu, self.get_parameter("imu_topic").get_parameter_value().string_value, self.imu_callback, 10)
        self.publisher = self.create_publisher(Odometry, self.get_parameter("odom_topic").get_parameter_value().string_value, 10)

        self.prev_time = None
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)

    def imu_callback(self, msg: Imu):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        self.prev_time = current_time

        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Simple dead-reckoning integration
        self.velocity += acc * dt
        self.position += self.velocity * dt

        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position = Point(x=self.position[0], y=self.position[1], z=self.position[2])
        odom_msg.pose.pose.orientation = msg.orientation  # orientation directly from IMU

        # Velocity
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = self.velocity[2]
        odom_msg.twist.twist.angular = msg.angular_velocity

        self.publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Odometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
