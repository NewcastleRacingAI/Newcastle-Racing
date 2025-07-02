import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from eufs_msgs.msg import PathWithBoundaries
#from eufs_sim.msg import CarState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from newcastle_racing_ai.utils.mpc_module import P, Node as MpcNode, PATH, calc_ref_trajectory_in_T_step, linear_mpc_control
import math
from math import atan2
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        # --------- ROS subscriptions and publishers ---------
        self.create_subscription(PathWithBoundaries, "/path", self.on_path, 10)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_odom, 10)
        self.create_subscription(Imu, "/ros_can/imu", self.on_imu, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, "/cmd", 10)

        # --------- MPC internal buffers ---------
        self.ref_path = None   # Reference path object
        self.sp = None         # Speed profile
        self.vehicle_state = None   # Current vehicle state [x, y, v, yaw]
        self.a_opt = None      # Warm start cache
        self.delta_opt = None
        self.max_speed = 1

        # Main control loop, 20 Hz
        self.create_timer(0.05, self.timer_callback)


    def on_imu(self, msg):
        #this may be needed for live running when can't get odom
        pass

    # --------- Callback: path subscription (from planner) ---------
    def on_path(self, msg):
        # cx, cy, cyaw = [], [], []
        cx, cy, cyaw = path_to_pose_array(msg.path)
        # for pose in pose_array:
        #     cx.append(pose.position.x)
        #     cy.append(pose.position.y)
        #     # Convert quaternion to yaw (Euler angle)
        #     q = pose.orientation
        #     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        #     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        #     yaw = math.atan2(siny_cosp, cosy_cosp)
        #     cyaw.append(yaw)
        if len(cx) < 2:
            self.get_logger().warn("Received path too short for MPC control.")
            return
        ck = [0.0] * len(cx)  # Curvature (not used, but required by PATH class)
        self.ref_path = PATH(cx, cy, cyaw, ck)
        self.sp = [P.target_speed] * len(cx)  # Constant speed profile; can be adapted as needed

    # --------- Callback: odometry/localization subscription ---------
    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v = math.hypot(vx, vy)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.vehicle_state = [x, y, v, yaw]

    # --------- Main timer callback (core MPC loop) ---------
    def timer_callback(self):
        # Only run MPC if both path and vehicle state are available
        if self.ref_path is None or self.vehicle_state is None:
            return

        node = MpcNode(*self.vehicle_state)
        z_ref, _ = calc_ref_trajectory_in_T_step(node, self.ref_path, self.sp)
        # Warm start support for improved optimization speed
        a_opt, delta_opt, *_ = linear_mpc_control(z_ref, self.vehicle_state, self.a_opt, self.delta_opt)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(delta_opt[0])
        #msg.drive.speed = float(self.vehicle_state[2] + a_opt[0] * P.dt)
        msg.drive.speed = float(self.vehicle_state[2])
        msg.drive.acceleration = float(a_opt[0])
        self.publisher.publish(msg)

        # Update warm start cache
        self.a_opt = a_opt
        self.delta_opt = delta_opt

def path_to_pose_array(path):
    """
    Converts a geometry_msgs/Point[] path to a list of objects with .position.x, .position.y, .orientation (quaternion).
    Each pose's orientation is set so that yaw (theta) points to the next point.
    """
    cx, cy, cyaw = [], [], []
    n = len(path)
    for i in range(n):
        x = path[i].x
        y = path[i].y
        _ = path[i].z
        if i < n - 1:
            dx = path[i + 1].x - x
            dy = path[i + 1].y - y
            theta = math.atan2(dy, dx)
        else:
            theta = math.atan2(path[i].y - path[i-1].y, path[i].x - path[i-1].x) if i > 0 else 0.0
 
        cx.append(x)
        cy.append(y)
        cyaw.append(theta)

    return cx, cy, cyaw

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
