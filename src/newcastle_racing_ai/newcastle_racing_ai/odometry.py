import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, PoseWithCovariance, Twist, TwistWithCovariance
import numpy as np
from newcastle_racing_ai_msgs.msg import NewCarState
from math import hypot, atan2 as math
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
        self.timer = self.create_timer(0.01, self.publish_data)

        self.prev_time = None
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.imu = Imu()
        # Kalman filter variables
        self.state_estimate
        self.error_matrix
        odom_msg = Odometry()
        car_state_msg = NewCarState()
        self.car_state_msg.vx = 0.0
        self.car_state_msg.vy = 0.0
        self.car_state_msg.velocity = 0.0
        self.car_state_msg.yaw = 0.0
        self.car_state_msg.angular_velocity = 0.0
        self.car_state_msg.heading = 0.0
        self.car_state_msg.x = 0.0
        self.car_state_msg.y = 0.0
        self.car_state_msg.distance = 0.0



        # list containing a list of position values and a yaw value
        # e.g. [[x, y, z], yaw]
        self.vehicle_state 

    def imu_callback(self, msg):
        self.imu = msg


    def publish_data(self):
        imu = self.imu
        #self.get_logger().info(f"Received IMU data: {imu.header.stamp.sec}.{imu.header.stamp.nanosec} - Acceleration: {imu.linear_acceleration.x}, {imu.linear_acceleration.y}, {imu.linear_acceleration.z}")
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
        
        self.vehicle_state_calc()

        
        odom_msg.header.stamp = imu.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position = Point(x=self.vehicle_state[0][0], y=self.vehicle_state[0][1], z=self.vehicle_state[0][2])
        odom_msg.pose.pose.orientation = imu.orientation
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = self.velocity[2]
        odom_msg.twist.twist.angular = imu.angular_velocity

        # variable to publish is self.vehicle_state
        # is an array containing [x, y, z], yaw
        # like this [ [x, y, z], yaw ]

        self.odom_publisher.publish(odom_msg)

    """
    Things to research/understand
    - Does ROS keep scope with old values are all values volatile upon each message
        - maybe publish message data then retrieve old data from /odom or /imu 
        - learn how ROS actually works
    - What data is available from odometries such as IMU or magnetometers
    - Particle and Kalman filters for error correction
    """

    def vehicle_state_calc(self):
        imu = self.imu
        # Point of the 3 position coordinates
        position_old = [0,0,0]
        position = [0, 0, 0]
        current_time = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            return
        
        dt = current_time - self.prev_time
       
        angular_velocity = imu.angular_velocity
        orientation = imu.orientation
       
        acceleration = np.array([
           imu.linear_acceleration.x,
           imu.linear_acceleration.y,
           imu.linear_acceleration.z
        ])   
        
        # slightly better velocity calculation
        linear_velocity_x_old = 0
        linear_velocity_y_old = 0
        linear_velocity_z_old = 0
        # V.old values for velocity
        linear_velocity_x_old = acceleration[0] * dt
        linear_velocity_y_old = acceleration[1] * dt
        linear_velocity_z_old = acceleration[2] * dt
        
        # V.new values for velocity
        linear_velocity_x_new = linear_velocity_x_old + acceleration[0] * dt
        linear_velocity_y_new = linear_velocity_y_old + acceleration[1] * dt
        linear_velocity_z_new = linear_velocity_z_old + acceleration[2] * dt     
        
        # slightly better position calculation
        position_old[0] = linear_velocity_x_old * dt
        position_old[1] = linear_velocity_y_old * dt
        position_old[2] = linear_velocity_z_old * dt
        
        distance_x = linear_velocity_x_old * dt + 0.5*acceleration[0]*(dt*dt)
        distance_y = linear_velocity_y_old * dt + 0.5*acceleration[1]*(dt*dt)
        distance_z = linear_velocity_z_old * dt + 0.5*acceleration[2]*(dt*dt)
        position[0] = position_old[0] + distance_x
        position[1] = position_old[1] + distance_y
        position[2] = position_old[2] + distance_z
        
        total_distance = hypo
        
        state_vector = np.array()

        # overwriting old variables
        linear_velocity_x_old = linear_velocity_x_new
        linear_velocity_y_old = linear_velocity_y_new
        linear_velocity_z_old = linear_velocity_z_new
        position_old[0] = position[0]
        position_old[1] = position[1]
        position_old[2] = position[2]
        
        
        # calculating yaw from sin(y)+cos(p) and cos(y)+cos(p)
        siny_cosp = 2.0 * ((orientation.w * orientation.z) + (orientation.x * orientation.y))
        cosy_cosp = -1.0 * ((orientation.y * orientation.y) + (orientation.z * orientation.z))
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.velocity[1] = linear_velocity_x_new
        self.velocity[2] = linear_velocity_y_new
        self.velocity[3] = linear_velocity_z_new
        self.vehicle_state = [position, yaw]   

        self.car_state_msg.vx = linear_velocity_x_new
        self.car_state_msg.vy = linear_velocity_y_new
        self.car_state_msg.velocity = hypot(linear_velocity_x_new, linear_velocity_y_new)
        self.car_state_msg.yaw = yaw
        self.car_state_msg.angular_velocity = 
        self.car_state_msg.heading = 0.0
        self.car_state_msg.x = position[0]
        self.car_state_msg.y = position[1]
        self.car_state_msg.distance =+ 



def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
