import rclpy
from rclpy.node import Node
from eufs_msgs.msg import  CanState
from nav_msgs.msg import Odometry
from newcastle_racing_ai_msgs.msg import Mission, MissionState
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from rclpy.duration import Duration

from .parameters import PARAMETERS

class Mission_Control(Node):
    """
    This node is listening for the go signal and mission selection from the ros_can node, which it passes on to the controller and allows the car to start driving.
    A separate node listens for any safety/ebs related signals. 
    """

    def __init__(self):
        super().__init__('Mission_Control')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        self._subscription = self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10)
        self._subscription = self.create_subscription(CanState, self.get_parameter("can_state_topic").value, self._on_state, 10)
        # to be sent to controller
        self._publisher_mission = self.create_publisher(Mission, self.get_parameter("mission_topic").value, 10)
        self._publisher_mission_state = self.create_publisher(MissionState, self.get_parameter("mission_state_topic").value, 10)
        # to be sent to the can node / VCU
        self._publisher_can = self.create_publisher(CanState, self.get_parameter("can_state_topic").value, 10)
        self._publisher_cmd = self.create_publisher(AckermannDriveStamped, "/cmd", 10)
        self._timer = self.create_timer(0.1, self.mission)  # 10 Hz

        # mission variables
        self.mission_state = MissionState.AS_OFF
        self.mission = Mission.AMI_NOT_SELECTED
        self.mission_state_msg = MissionState()
        self.mission_msg = Mission()
        self.can_reply = CanState()
        self.odom = Odometry()
        self.cmd = AckermannDriveStamped()
        self.initial_odom = Odometry()
        self.initial_time = None
        # for a quick start
        self.full_steam_ahead = AckermannDriveStamped()
        self.full_steam_ahead.drive.steering_angle = 0.0
        self.full_steam_ahead.drive.acceleration = 1.0
        self.full_steam_ahead.drive.speed = 1.0
        # for quick brake stop
        self.full_brake_stop = AckermannDriveStamped()
        self.full_brake_stop.drive.steering_angle = 0.0
        self.full_brake_stop.drive.acceleration = -1.0
        self.full_brake_stop.drive.speed = 0.0
        # for demonstration mission
        self.steering = AckermannDriveStamped()
        self.steering.drive.steering_angle = 0.0
        self.steering.drive.steering_angle_velocity = 0.2
        self.steering.drive.speed = 0.5
        self.steering.drive.acceleration = 0.0
      


    def _on_state(self, msg):
        self.get_logger().info('Received: "%s"' % type(msg))
        if msg.as_state != MissionState.AS_OFF:
            self.get_logger().info('Received mission state: %s' % msg.as_state)
            mission_state_msg = MissionState()
            self.mission_state = msg.as_state
            mission_state_msg.mission_state = self.mission_state
        if self.mission != Mission.AMI_NOT_SELECTED:
            self.get_logger().info('Mission already selected: %s' % self.mission)
            return
        else:
            self.mission = msg.ami_state
            mission_msg = Mission()
            mission_msg.mission = self.mission
            #self._publisher_mission.publish(self.mission_msg)

    def _on_odom(self, msg):
        self.odom = msg
        self.get_logger().info('Received odometry data: %s' % self.odom)
        # add the distance tracking and whether the car is stopped or driving here to be used in demo mission
        
        

    def mission(self):
        """
        This is a state machine that controls the mission.
        """
        if self.mission == Mission.AMI_NOT_SELECTED:
            self.get_logger().info('No mission selected, waiting for signal...')
            self.initial_odom = self.odom
            self.initial_time = self.get_clock().now()
            return
        # Static Mission A
        # still need to find out how to get the rpm
        elif self.mission == Mission.AMI_DDT_INSPECTION_A:
            self.get_logger().info('Starting inspection mission A...')
            if self.mission_state == MissionState.AS_DRIVING:
                self.get_logger().info('AMI_DDT_INSPECTION_A is ready, starting driving...')
                self.get_logger().info('Time = %s' % (self.get_clock().now() - self.initial_time))
                # For 0-10 s drive full steam ahead, for >10s apply brake stop
                if (self.get_clock().now() - self.initial_time) < Duration(seconds=10):
                    self._publisher_cmd.publish(self.full_steam_ahead)
                else:
                    self.mission_state = MissionState.AS_FINISHED
                    self.can_reply.AS_State = self.mission_state
                    self.can_reply.ami_state = self.mission
                    self._publisher_cmd.publish(self.full_brake_stop)
                    self._publisher_can.publish(self.can_reply)
        # Static Mission B
        # still need to find out how to get the rpm
        elif self.mission == Mission.AMI_DDT_INSPECTION_B:
            self.get_logger().info('Starting inspection mission B...')
            if self.mission_state == MissionState.AS_DRIVING:
                self.get_logger().info('AMI_DDT_INSPECTION_B is ready, starting driving...')
                self.get_logger().info('Time = %s' % (self.get_clock().now() - self.initial_time))
                # For 0-10 s drive full steam ahead, for >10s apply brake stop
                if (self.get_clock().now() - self.initial_time) < Duration(seconds=10):
                    self._publisher_cmd.publish(self.full_steam_ahead)
                else:
                    self.mission_state = MissionState.AS_FINISHED
                    self.can_reply.AS_State = self.AS_EMERGENCY_BRAKE
                    self.can_reply.ami_state = self.mission
                    self._publisher_can.publish(self.can_reply)
        # Demonstration Mission
        elif self.mission == Mission.AMI_AUTONOMOUS_DEMO:
            self.get_logger().info('Starting Demonstration Mission...')
            self.demo_state = 0
            if self.mission_state == MissionState.AS_DRIVING:
                self.get_logger().info('AMI_Autonomous_Demo is ready, starting driving...')
                # turn left for 10s
                if (self.get_clock().now() - self.initial_time) < Duration(seconds=10):
                    if self.demo_state == 0:
                        self.steering.drive.steering_angle = -1.0
                        self._publisher_cmd.publish(self.steering)
                        self.demo_state = 1
                # turn right for 10s
                elif (self.get_clock().now() - self.initial_time) < Duration(seconds=20):
                    if self.demo_state == 1:
                        self.steering.drive.steering_angle = 1.0
                        self._publisher_cmd.publish(self.steering)
                        self.demo_state = 2
                #turn straight for 10s
                elif (self.get_clock().now() - self.initial_time) < Duration(seconds=30):
                    if self.demo_state == 2:
                        self.steering.drive.steering_angle = 0.0
                        self._publisher_cmd.publish(self.steering)
                        self.demo_state = 3
                elif (self.get_clock().now() - self.initial_time) > Duration(seconds=40):
                    if self.demo_state == 3:
                        self._publisher_mission_state(self.mission_state_msg)
                        self.demo_state = 4
                    elif self.demo_state == 4:
                        x_dist = self.odom.pose.pose.position.x - self.initial_odom.pose.pose.position.x
                        y_dist = self.odom.pose.pose.position.y - self.initial_odom.pose.pose.position.y
                        self.total_distance = np.sqrt(x_dist**2 + y_dist**2)
                        if self.total_distance > 10:
                            self.mission_state_msg.mission_state = MissionState.AS_BRAKE
                            self._publisher_mission_state(self.mission_state_msg)
                            
                        
                
        
    

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Mission_Control()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()