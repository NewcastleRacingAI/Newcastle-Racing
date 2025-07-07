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
from newcastle_racing_ai_msgs.msg import MissionState
from .parameters import PARAMETERS

class Controller(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        # --------- ROS subscriptions and publishers ---------
        self.create_subscription(PathWithBoundaries,  self.get_parameter("path_topic").value, self.on_path, 10)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.on_odom, 10)
        self.create_subscription(MissionState, self.get_parameter("mission_state_topic").value, self.on_state_command, 10)
        self.control_publisher = self.create_publisher(AckermannDriveStamped, "/cmd", 10)

        # --------- MPC internal buffers ---------
        self.ref_path = None   # Reference path object
        self.sp = None         # Speed profile
        self.vehicle_state = None   # Current vehicle state [x, y, v, yaw]
        self.a_opt = None      # Warm start cache
        self.delta_opt = None
        self.max_speed = 1

        # --------- Mission Control state management ---------
        self.mission_state = MissionState()
        self.mission_state.mission_state = MissionState.AS_OFF  # Initial state
        self.full_brake_stop = AckermannDriveStamped()
        self.full_brake_stop.drive.steering_angle = 0.0
        self.full_brake_stop.drive.acceleration = -1.0
        self.full_brake_stop.drive.speed = 0.0
        self.cmd = AckermannDriveStamped()
        

        # Main control loop, 20 Hz
        self.create_timer(0.2, self.timer_callback)

    def on_state_command(self, msg):
        """
        Callback to handle drive commands from the mission control node.
        This sets the drive flag to True when the car should start driving.
        """
        self.mission_state.mission_state = msg.mission_state
        if msg.mission_state == MissionState.AS_DRIVING:
            self.get_logger().info("Received drive command, starting MPC control.")
        elif msg.mission_state == MissionState.AS_BRAKE:
            self.get_logger().info("Received brake command, controlled braking starting.")
        elif msg.mission_state == MissionState.AS_FINISHED:
            self.get_logger().info("Received finish command, braking then stopping MPC control.")


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
            self.get_logger().warn("MPC control not running: missing path or vehicle state.")
            return

        node = MpcNode(*self.vehicle_state)
        z_ref, _ = calc_ref_trajectory_in_T_step(node, self.ref_path, self.sp)
        # Warm start support for improved optimization speed
        a_opt, delta_opt, *_ = linear_mpc_control(z_ref, self.vehicle_state, self.a_opt, self.delta_opt)

        
        self.cmd.header.stamp = self.get_clock().now().to_msg()
        self.cmd.drive.steering_angle = float(delta_opt[0])
        #msg.drive.speed = float(self.vehicle_state[2] + a_opt[0] * P.dt)
        self.cmd.drive.speed = float(self.vehicle_state[2])
        self.cmd.drive.acceleration = float(a_opt[0])
        # only publish if mission control has set the drive flag
        if self.mission_state.mission_state == MissionState.AS_OFF or self.mission_state.mission_state == MissionState.AS_READY or self.mission_state.mission_state == MissionState.AS_FINISHED:
            self.get_logger().info("System not ready to drive, not publishing control command.")
        elif self.mission_state.mission_state == MissionState.AS_DRIVING:
            # If driving, publish the control command
            self.get_logger().info(f"Publishing control command: speed={self.cmd.drive.speed}, steering={self.cmd.drive.steering_angle}, acceleration={self.cmd.drive.acceleration}")
            self.control_publisher.publish(self.cmd)
        elif self.mission_state.mission_state == MissionState.AS_BRAKE or self.mission_state.mission_state == MissionState.AS_EMERGENCY_BRAKE:
            # If not driving, send full brake stop command
            self.cmd.drive.speed = 0.0
            self.cmd.drive.acceleration = -1.0
            self.control_publisher.publish(self.cmd)
            self.get_logger().info("Braking command sent, stopping vehicle.")
            if self.vehicle_state[2] < 0.1:
                # If vehicle is stopped, set mission state to finished
                self.mission_state.mission_state = MissionState.AS_READY
                self.get_logger().info("Vehicle stopped, setting mission state to ready.")

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
