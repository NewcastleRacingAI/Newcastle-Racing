import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, PoseWithCovariance, Twist, TwistWithCovariance
import numpy as np
from newcastle_racing_ai_msgs.msg import NewCarState
from math import hypot, atan2, asin
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
        self.car_state_publisher = self.create_publisher(NewCarState, self.get_parameter("car_state_topic").get_parameter_value().string_value, 10)
        self.timer = self.create_timer(0.05, self.publish_data)

        self.imu_buffer = []  # Buffer for incoming IMU messages
        self.prev_time = None
        self.position = np.zeros(3)
        self.position_old = np.zeros(3)
        self.velocity = np.zeros(3)
        self.imu = None
        self.imu_avg_count = 0
        self.distance = 0.0
        # Kalman filter variables
        # self.state_estimate
        # self.error_matrix
        self.odom_msg = Odometry()
        self.car_state_msg = NewCarState()
        self.car_state_msg.vx = 0.0
        self.car_state_msg.vy = 0.0
        self.car_state_msg.velocity = 0.0
        self.car_state_msg.yaw = 0.0
        self.car_state_msg.angular_velocity = 0.0
        self.car_state_msg.x = 0.0
        self.car_state_msg.y = 0.0
        self.car_state_msg.distance = 0.0

        # Complementary filter and deadband threshold tunable parameters
        self.yaw = 0.0
        self.ACC_THRESHOLD = 0.3 # to stop stationary noise 0.3
        self.VEL_THRESHOLD = 0.1 # to stop stationary noise 0.3
        self.ANGVEL_THRESHOLD = 0.01
        self.MAX_IMU_BUFFER = 100
        self.odometry_fudge_factor_x = 2.20
        self.odometry_fudge_factor_y = 2.20
        self.odometry_fudge_factor = np.array([self.odometry_fudge_factor_x, self.odometry_fudge_factor_y, 1.0])
        

        # list containing a list of position values and a yaw value
        # e.g. [[x, y, z], yaw]
        self.vehicle_state = [[0.0, 0.0, 0.0], 0.0]  # [position, yaw]
        self.prev_imu_time = None

        self.last_ang_vel = np.zeros(3)  # <-- Add this line
        self.last_orientation = [0.0, 0.0, 0.0, 1.0]  # <-- Also initialize orientation for safety

    def imu_callback(self, msg):
        self.last_imu_msg = msg
        # Get current time from IMU message
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_imu_time is None:
            self.prev_imu_time = current_time
            # Initialize yaw from orientation
            self.yaw = self.quaternion_to_yaw([
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            ])
            return

        dt = current_time - self.prev_imu_time
        self.prev_imu_time = current_time

        # Get acceleration and angular velocity as numpy arrays
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        ang_vel = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        # Apply deadband
        acc = self.apply_deadband(acc, self.ACC_THRESHOLD)
        ang_vel = self.apply_deadband(ang_vel, self.ANGVEL_THRESHOLD)

        # Integrate only if not stationary
        is_stationary = np.linalg.norm(acc[:2]) < self.ACC_THRESHOLD
        #self.get_logger().info(f"IMU Data: Acc={np.linalg.norm(acc[:2])}, yaw= {self.yaw}, Stationary={is_stationary}")

        if is_stationary:
            self.velocity[:2] = 0
            # Do NOT update position when stationary
        else:
            self.velocity += acc * dt
            if self.velocity[0] < 0:
                self.velocity[0] = 0.0  # No reverse

            # Transform velocity from robot frame to world frame using current yaw
            v_world = np.zeros(3)
            v_world[0] = self.velocity[0] * np.cos(self.yaw) - self.velocity[1] * np.sin(self.yaw)
            v_world[1] = self.velocity[0] * np.sin(self.yaw) + self.velocity[1] * np.cos(self.yaw)
            v_world[2] = self.velocity[2]

            self.position += v_world * self.odometry_fudge_factor * dt
            self.distance += hypot(self.velocity[0] * self.odometry_fudge_factor_x, self.velocity[1] * self.odometry_fudge_factor_y) * dt

            # Complementary filter for yaw
            alpha = 0.98
            gyro_yaw = self.yaw + ang_vel[2] * dt
            quat_yaw = self.quaternion_to_yaw([
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            ])
            self.yaw = alpha * gyro_yaw + (1 - alpha) * quat_yaw

        self.last_orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        self.last_ang_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

    def publish_data(self):
        # Just publish the current state
        self.car_state_msg.vx = self.clamp_small(self.velocity[0])
        self.car_state_msg.vy = self.clamp_small(self.velocity[1])
        self.car_state_msg.velocity = self.clamp_small(hypot(self.velocity[0], self.velocity[1]))
        self.car_state_msg.x = self.position[0]
        self.car_state_msg.y = self.position[1]
        self.car_state_msg.yaw = self.yaw
        self.car_state_msg.angular_velocity = self.last_ang_vel[2]
        self.car_state_msg.distance = self.distance

        if hasattr(self, 'last_imu_msg'):
            self.odom_msg.header.stamp = self.last_imu_msg.header.stamp
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'
        self.odom_msg.pose.pose.position = Point(x=self.position[0], y=self.position[1], z=self.position[2])
        self.odom_msg.pose.pose.orientation.x = self.last_orientation[0]
        self.odom_msg.pose.pose.orientation.y = self.last_orientation[1]
        self.odom_msg.pose.pose.orientation.z = self.last_orientation[2]
        self.odom_msg.pose.pose.orientation.w = self.last_orientation[3]
        self.odom_msg.twist.twist.linear.x = self.velocity[0]
        self.odom_msg.twist.twist.linear.y = self.velocity[1]
        self.odom_msg.twist.twist.linear.z = self.velocity[2]
        self.odom_msg.twist.twist.angular.x = self.last_ang_vel[0]
        self.odom_msg.twist.twist.angular.y = self.last_ang_vel[1]
        self.odom_msg.twist.twist.angular.z = self.last_ang_vel[2]

        self.car_state_publisher.publish(self.car_state_msg)
        self.odom_publisher.publish(self.odom_msg)

        self.imu_buffer.clear()  # Clear buffer for next period

    def clamp_small(self, val):
        return 0.0 if abs(val) < self.VEL_THRESHOLD else val

    def apply_deadband(self, arr, threshold):
        arr[np.abs(arr) < threshold] = 0
        return arr

    def quaternion_to_yaw(self, q):
        # q: [x, y, z, w]
        x, y, z, w = q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return atan2(siny_cosp, cosy_cosp)

    """
    Things to research/understand
    - Does ROS keep scope with old values are all values volatile upon each message
        - maybe publish message data then retrieve old data from /odom or /imu 
        - learn how ROS actually works
    - What data is available from odometries such as IMU or magnetometers
    - Particle and Kalman filters for error correction
    """


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
