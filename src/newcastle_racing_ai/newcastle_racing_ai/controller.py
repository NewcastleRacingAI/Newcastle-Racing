import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from eufs_msgs.msg import PathWithBoundaries, CarState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from newcastle_racing_ai.utils.mpc_module import P, Node as MpcNode, PATH, calc_ref_trajectory_in_T_step, linear_mpc_control
import math
from math import atan2
import numpy as np
from newcastle_racing_ai_msgs.msg import MissionState, NewCarState
from .parameters import PARAMETERS

class Controller(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        # --------- ROS subscriptions and publishers ---------
        self.create_subscription(PathWithBoundaries,  self.get_parameter("path_topic").value, self.on_path, 10)
        #self.create_subscription(CarState, self.get_parameter("car_state_topic").value, self.on_car_state, 10)
        self.create_subscription(NewCarState, self.get_parameter("car_state_topic").value, self.on_car_state, 10)
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
        self.path = PathWithBoundaries(). path = []  # Initialize path as an empty list
        #self.odom = Odometry()
        self.car_state = CarState()

        # Main control loop, 20 Hz
        self.create_timer(0.05, self.timer_callback)

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
        self.path = msg.path
        #self.get_logger().info(f"Received path with {len(self.path)} points.")
        

    # --------- Callback: odometry/localization subscription ---------
    def on_car_state(self, msg):
        self.car_state = msg
        
    
    # --------- Main timer callback (core MPC loop) ---------
    def timer_callback(self):
        if len(self.path) < 2:
            self.get_logger().warn("Path is too short. Skipping control.A")
            return

        if self.mission_state.mission_state not in [MissionState.AS_DRIVING]:
            self.get_logger().info("Not in DRIVING state. Not publishing command.")
            return


        # --- Get current vehicle state ---
        x, y, v, yaw = self.car_state.x, self.car_state.y, self.car_state.velocity, self.car_state.yaw
        target_speed = 0.25  # [m/s]

        # --- Get target point from path ---
        target_point = self.path[1]  # Next point
        dx = target_point.x - x
        dy = target_point.y - y

        # --- Calculate heading error ---
        angle_to_target = math.atan2(dy, dx)
        heading_error = angle_to_target - yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))  # Normalize [-pi, pi]

        # --- Steering angle ---
        steering_angle = heading_error  # Proportional to heading error (simple P controller)

        # --- Acceleration to reach target speed ---
        speed_error = target_speed - v
        acceleration = 1.5 * speed_error  # Proportional control
        acceleration = max(min(acceleration, 2.0), -1.0)  # Clamp acceleration

        # --- Build and publish command ---
        self.cmd.header.stamp = self.get_clock().now().to_msg()
        self.cmd.drive.steering_angle = steering_angle
        self.cmd.drive.acceleration = acceleration

        self.get_logger().info(
            f"Cmd: speed={v:.2f}, accel={acceleration:.2f}, steering={steering_angle:.2f}, target=({target_point.x:.2f}, {target_point.y:.2f})"
        )
        self.control_publisher.publish(self.cmd)


    def path_to_pose_array(self):
        """
        Converts a geometry_msgs/Point[] path to a list of objects with .position.x, .position.y, .orientation (quaternion).
        Each pose's orientation is set so that yaw (theta) points to the next point.
        """
        cx, cy, cyaw = [], [], []
        n = len(self.path)
        for i in range(n):
            x = self.path[i].x
            y = self.path[i].y
            _ = self.path[i].z
            if i < n - 1:
                dx = self.path[i + 1].x - x
                dy = self.path[i + 1].y - y
                theta = math.atan2(dy, dx)
            else:
                theta = math.atan2(self.path[i].y - self.path[i-1].y, self.path[i].x - self.path[i-1].x) if i > 0 else 0.0
    
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
