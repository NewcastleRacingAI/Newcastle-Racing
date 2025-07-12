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
        self.ACC_THRESHOLD = 0.1 
        self.VEL_THRESHOLD = 0.1
        self.ANGVEL_THRESHOLD = 0.01
        self.MAX_IMU_BUFFER = 100
        

        # list containing a list of position values and a yaw value
        # e.g. [[x, y, z], yaw]
        self.vehicle_state = [[0.0, 0.0, 0.0], 0.0]  # [position, yaw]

    def imu_callback(self, msg):
        # Buffer each incoming IMU message
        self.imu_buffer.append(msg)
        if len(self.imu_buffer) > self.MAX_IMU_BUFFER:
            self.imu_buffer = self.imu_buffer[-self.MAX_IMU_BUFFER:]

    def publish_data(self):
        if not self.imu_buffer:
            return  # No IMU data to process

        # Average all buffered IMU messages
        n = len(self.imu_buffer)
        avg_acc = np.zeros(3)
        avg_ang_vel = np.zeros(3)
        avg_orientation = np.zeros(4)  # [x, y, z, w]
        for imu in self.imu_buffer:
            avg_acc += np.array([
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z
            ])
            avg_ang_vel += np.array([
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z
            ])
            avg_orientation += np.array([
                imu.orientation.x,
                imu.orientation.y,
                imu.orientation.z,
                imu.orientation.w
            ])
        avg_acc /= n
        avg_ang_vel /= n
        avg_orientation /= n

        # Apply deadband filtering
        avg_acc = self.apply_deadband(avg_acc, self.ACC_THRESHOLD)
        avg_ang_vel = self.apply_deadband(avg_ang_vel, self.ANGVEL_THRESHOLD)

        # Use the timestamp of the last IMU message
        imu = self.imu_buffer[-1]
        current_time = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            self.imu_buffer.clear()
            # Initialize yaw from orientation
            self.yaw = self.quaternion_to_yaw(avg_orientation)
            return

        dt = current_time - self.prev_time
        self.prev_time = current_time

        # --- Complementary filter for yaw ---
        alpha = 0.98  # blending factor, tune as needed (0.98 is common)
        # Integrate gyro z (yaw rate)
        gyro_yaw = self.yaw + avg_ang_vel[2] * dt
        # Yaw from orientation quaternion
        quat_yaw = self.quaternion_to_yaw(avg_orientation)
        # Complementary filter
        self.yaw = alpha * gyro_yaw + (1 - alpha) * quat_yaw

        is_stationary = np.linalg.norm(avg_acc[:2]) < self.ACC_THRESHOLD and np.linalg.norm(avg_ang_vel[:2]) < self.ANGVEL_THRESHOLD

        if is_stationary:
            # Aggressively zero velocity
            self.velocity[:2] = 0
            self.car_state_msg.vx = 0.0
            self.car_state_msg.vy = 0.0
            self.car_state_msg.velocity = 0.0
            # Optionally, do NOT update position/yaw when stationary
            # (comment out the lines that update self.position and self.yaw when is_stationary is True)
        else:
            # Only integrate when moving
            self.velocity += avg_acc * dt
            self.position += self.velocity * dt
            # Complementary filter for yaw
            gyro_yaw = self.yaw + avg_ang_vel[2] * dt
            quat_yaw = self.quaternion_to_yaw(avg_orientation)
            alpha = 0.98
            self.yaw = alpha * gyro_yaw + (1 - alpha) * quat_yaw
            self.car_state_msg.vx = self.clamp_small(self.velocity[0])
            self.car_state_msg.vy = self.clamp_small(self.velocity[1])
            self.car_state_msg.velocity = self.clamp_small(hypot(self.velocity[0], self.velocity[1]))

        # Always publish the last position/yaw, but don't update them when stationary
        self.car_state_msg.x = self.position[0]
        self.car_state_msg.y = self.position[1]
        self.car_state_msg.yaw = self.yaw
        self.car_state_msg.angular_velocity = avg_ang_vel[2]
        self.car_state_msg.distance += hypot(self.velocity[0] * dt, self.velocity[1] * dt)

        self.odom_msg.header.stamp = imu.header.stamp
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'
        self.odom_msg.pose.pose.position = Point(x=self.position[0], y=self.position[1], z=self.position[2])
        self.odom_msg.pose.pose.orientation.x = avg_orientation[0]
        self.odom_msg.pose.pose.orientation.y = avg_orientation[1]
        self.odom_msg.pose.pose.orientation.z = avg_orientation[2]
        self.odom_msg.pose.pose.orientation.w = avg_orientation[3]
        self.odom_msg.twist.twist.linear.x = self.velocity[0]
        self.odom_msg.twist.twist.linear.y = self.velocity[1]
        self.odom_msg.twist.twist.linear.z = self.velocity[2]
        self.odom_msg.twist.twist.angular.x = avg_ang_vel[0]
        self.odom_msg.twist.twist.angular.y = avg_ang_vel[1]
        self.odom_msg.twist.twist.angular.z = avg_ang_vel[2]

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
