import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from newcastle_racing_ai_msgs.msg import ControlCommand, PathWithBoundaries, EBS
from newcastle_racing_ai.utils.transformations import quaternion_to_euler
#from newcastle_racing_ai.utils.mpc_module import 
from .parameters import PARAMETERS

class Controller(Node):

    def __init__(self):
        super().__init__('Controller')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        #self._subscription = self.create_subscription(PoseArray, self.get_parameter("path").value, self._on_path, 10)
        self.create_subscription(PathWithBoundaries, '/path', self._on_path, 10)
        self.create_subscription(Imu, '/nrfai/imu', self.imu_callback, 10)
        self.create_subscription(EBS, '/nrfai/ebs_topic', self.ebs_callback, 10)
        #self._publisher = self.create_publisher(AckermannDriveStamped, self.get_parameter("cmd_topic").value, 10)
        self.control_publisher = self.create_publisher(ControlCommand, '/fsds/control_command', 10)
        #self.timer = self.create_timer(timer_period, self._timer_callback)
        self.path = [(0,0,0)]
        self.ebs = False
        self.ebs_msg = ControlCommand(throttle=0, steering=0, brake=1, handbrake=True)
    
    def ebs_callback(self, msg):
        self.ebs = msg.ebs_stop
        self.control_publisher.publish(self.ebs_msg)

    def _on_path(self, msg):
        self.get_logger().info('Received: "%s"' % type(msg))
        #check for ebs stop command first
        if self.ebs:
            self.control_publisher.publish(self.ebs_msg)
        else:
            self.path = msg.path # array of points (x,y,z)
            #control method goes here 
            #throttle, steering, brake = call_to_mpc_module
            # and calls publish_control()
            #self.publish_control(throttle, steering, brake)
            #should also publish to can_bus_message...?
            #will need to add a live / sim method as the live method will need to write to the can bus. 

    def imu_callback(self, imu_msg):
        _, _, yaw = quaternion_to_euler(imu_msg.orientation)
        self.car_direction = np.array([np.cos(yaw), np.sin(yaw)])
        self.check_and_compute_path()

    def publish_control(self, throttle, steering, brake):
        # float64 throttle # [-] range : (0, 1)
        # float64 steering # [-] range : (-1, 1)
        # float64 brake # # [-] range : (0, 1)
        msg = ControlCommand(throttle=throttle, steering=steering, brake=brake)
        self.control_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Controller()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
