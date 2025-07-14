import rclpy
from rclpy.node import Node
from eufs_msgs.msg import  CanState, WheelSpeedsStamped
from nav_msgs.msg import Odometry
from newcastle_racing_ai_msgs.msg import Mission, MissionState, NewCarState
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import numpy as np
from rclpy.duration import Duration
import math

from .parameters import PARAMETERS

class Mission_Control(Node):
    """
    This node is listening for the go signal and mission selection from the ros_can node, which it passes on to the controller and allows the car to start driving.
    A separate node listens for any safety/ebs related signals. 
    """

    def __init__(self):
        super().__init__('Mission_Control')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        self._subscription = self.create_subscription(WheelSpeedsStamped, self.get_parameter("can_wheel_speed_topic").value, self._on_wheel_speeds, 10)
        #self._subscription = self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10)
        self._subscription = self.create_subscription(CanState, self.get_parameter("can_state_topic").value, self._on_state, 10)
        self._subscription = self.create_subscription(NewCarState, self.get_parameter("car_state_topic").get_parameter_value().string_value, self._on_car_state, 10)
        # to be sent to controller
        self._publisher_mission = self.create_publisher(Mission, self.get_parameter("mission_topic").value, 10)
        self._publisher_mission_state = self.create_publisher(MissionState, self.get_parameter("mission_state_topic").value, 10)
        # to be sent to the can node / VCU
        # the ebs internal request client via ros_can
        self.cli = self.create_client(Trigger, self.get_parameter("can_ebs_request_service").value)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ros_can ebs service...')
        self._publisher_can_complete = self.create_publisher(Bool, self.get_parameter("can_mission_complete_topic").value, 10)
        self._publisher_cmd = self.create_publisher(AckermannDriveStamped, "/cmd", 10)
        self._timer = self.create_timer(0.1, self.mission)  # 10 Hz

        # mission variables
        self.mission_state = MissionState.AS_OFF
        self.mission_type = Mission.AMI_NOT_SELECTED
        self.mission_state_msg = MissionState()
        self.mission_msg = Mission()
        self.can_reply = False # was can_state
        self.odom = Odometry()
        self.wheel_speeds = WheelSpeedsStamped()
        self.average_wheel_speed = 0.0
        self.cmd = AckermannDriveStamped()
        self.initial_odom = Odometry()
        self.initial_time = None
        self.demo_state = 0
        self.drive_state = 0
        self.distance = 0.0
        # for a quick start
        self.full_steam_ahead = AckermannDriveStamped()
        self.full_steam_ahead.drive.steering_angle = 0.0
        self.full_steam_ahead.drive.acceleration = 10.0
        self.full_steam_ahead.drive.speed = 1.0
        # for quick brake stop
        self.full_brake_stop = AckermannDriveStamped()
        self.full_brake_stop.drive.steering_angle = 0.0
        self.full_brake_stop.drive.acceleration = -10.0
        self.full_brake_stop.drive.speed = 0.0
        # for demonstration mission
        self.steering = AckermannDriveStamped()
        self.steering.drive.steering_angle = 0.0
        self.steering.drive.steering_angle_velocity = 0.01
        self.steering.drive.speed = 0.0
        self.steering.drive.acceleration = 0.0
      
    # --------- Callback: odometry/localization subscription ---------
    def _on_car_state(self, msg):
        # this is from our odometry node so results are currently unreliable
        self.vehicle_state = [msg.x, msg.y, msg.velocity, msg.yaw]
        self.distance = msg.distance


    def call_ebs_service(self):
        """ Call the EBS service of ros_can node to request an internal emergency brake stop.
        """        
        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.task.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"EBS Service response: success={future.result().success}, message={future.result().message}")
        else:
            self.get_logger().error("EBS Service call failed")

    def _on_wheel_speeds(self, msg):
        self.wheel_speeds = msg
        self.average_wheel_speed = (msg.speeds.lf_speed + msg.speeds.rf_speed + msg.speeds.lb_speed + msg.speeds.rb_speed) / 4.0
        #self.get_logger().info('Received wheel speeds: %s' % self.average_wheel_speed)


    def _on_state(self, msg):
        #self.get_logger().info('Received: "%s"' % type(msg))
        # this should only trigger the if statements once per mission run
        if msg.as_state != MissionState.AS_OFF and self.mission_state != msg.as_state:
            self.get_logger().info('Received mission state: %s' % msg.as_state)
            self.mission_state = msg.as_state
            self.mission_state_msg.mission_state = self.mission_state
        if msg.ami_state != Mission.AMI_NOT_SELECTED and self.mission_type != msg.ami_state:
            self.get_logger().info('Mission selected: %s' % self.mission_type)
            self.mission_type = msg.ami_state
            self.mission_msg.mission = self.mission_type
            # mission publisher is primarily for the path planner node
            self._publisher_mission.publish(self.mission_msg)

        
    def mission(self):
        """
        This is a state machine that controls the mission.
        """
        if self.mission_type == Mission.AMI_NOT_SELECTED:
            self.get_logger().info('No mission selected, waiting for signal...')
            return
        # Static Mission A
        # still need to find out how to get the rpm
        elif self.mission_type == Mission.AMI_DDT_INSPECTION_A:
            if self.mission_state == MissionState.AS_READY:
                self.get_logger().info('Starting inspection mission A...')
                self.initial_distance = self.distance
                self.initial_time = self.get_clock().now()
            if self.mission_state == MissionState.AS_DRIVING:
                self.get_logger().info('AMI_DDT_INSPECTION_A is ready, starting driving...')
                self.get_logger().info('Time = %s' % (self.get_clock().now() - self.initial_time))
                # For 0-10 s drive full steam ahead until 200rpm reached, for >10s apply brake stop
                if (self.get_clock().now() - self.initial_time) < Duration(seconds=10) and self.average_wheel_speed <= 200:
                    self._publisher_cmd.publish(self.full_steam_ahead)
                else:
                    self.mission_state = MissionState.AS_FINISHED
                    #self.can_reply.as_state = self.mission_state
                    #self.can_reply.ami_state = self.mission_type
                    self._publisher_cmd.publish(self.full_brake_stop)
                    self.can_reply = True
                    self._publisher_can_complete.publish(self.can_reply)

        # Static Mission B
        elif self.mission_type == Mission.AMI_DDT_INSPECTION_B:
            if self.mission_state == MissionState.AS_READY:
                self.get_logger().info('Starting inspection mission B...')
                self.initial_distance = self.distance
                self.initial_time = self.get_clock().now()
            if self.mission_state == MissionState.AS_DRIVING:
                self.get_logger().info('AMI_DDT_INSPECTION_B is ready, starting driving...')
                elapsed_time = (self.get_clock().now() - self.initial_time).nanoseconds / 1e9
                self.get_logger().info('Time = %.2f seconds' % elapsed_time)
                # For 0-10 s drive full steam ahead, for >10s apply brake stop
                if (self.get_clock().now() - self.initial_time) < Duration(seconds=10) and self.average_wheel_speed <= 50:
                    self._publisher_cmd.publish(self.full_steam_ahead)
                else:
                    self.mission_state = MissionState.AS_EMERGENCY_BRAKE
                    self.call_ebs_service()
                    self.get_logger().info('Emergency brake applied, stopping the car...')

        # Demonstration Mission
        elif self.mission_type == Mission.AMI_AUTONOMOUS_DEMO:
            if self.mission_state == MissionState.AS_READY:
                self.get_logger().info('Starting Demonstration Mission...')
                self.initial_time = self.get_clock().now()
            if self.mission_state == MissionState.AS_DRIVING:
                elapsed_time = (self.get_clock().now() - self.initial_time).nanoseconds / 1e9
                self.get_logger().info('Time = %.2f seconds' % elapsed_time)
                self.get_logger().info('AMI_Autonomous_Demo is ready, starting driving...')
                # turn left for 10s
                if (self.get_clock().now() - self.initial_time) < Duration(seconds=5):
                    # demo states progress after cmd sent, so cmd is only sent once per state
                    # if self.demo_state == 0:
                    self.get_logger().info('-------Turning left------')
                    self.steering.drive.steering_angle = 1.0
                    self._publisher_cmd.publish(self.steering)
                    #self.demo_state = 1
                # turn right for 10s
                elif (self.get_clock().now() - self.initial_time) < Duration(seconds=10):
                    #if self.demo_state == 1:
                    self.get_logger().info('--------Turning right --------')
                    self.steering.drive.steering_angle = -1.0
                    self._publisher_cmd.publish(self.steering)
                    #self.demo_state = 2
                #turn straight for 10s
                elif (self.get_clock().now() - self.initial_time) < Duration(seconds=15):
                   # if self.demo_state == 2:
                    self.get_logger().info('------Turning straight-------')
                    self.steering.drive.steering_angle = 0.0
                    self._publisher_cmd.publish(self.steering)
                    self.demo_state = 3
                elif (self.get_clock().now() - self.initial_time) > Duration(seconds=20):
                    # start measuring distance travelled
                    self.total_distance = self.distance - self.initial_distance
                    if self.demo_state == 3:
                        # tell controller to take over the driving
                        self.get_logger().info('------Starting driving-------')
                        self._publisher_mission_state.publish(self.mission_state_msg)
                        self.demo_state = 4
                    elif self.demo_state == 4:
                        #checking to see if travelled  first 10m
                        if self.total_distance > 10 and self.total_distance < 20:
                            if self.vehicle_state[2] > 0.1:
                                self.mission_state_msg.mission_state = MissionState.AS_BRAKE
                                self._publisher_mission_state.publish(self.mission_state_msg)
                            elif self.vehicle_state[2] < 0.1:
                                self.mission_state_msg.mission_state = MissionState.AS_DRIVING
                                self._publisher_mission_state.publish(self.mission_state_msg)
                                #reset the distance travelled
                                self.initial_odom = self.odom
                                self.demo_state = 5
                    # start moving the next 10m
                    elif self.demo_state == 5:
                        if self.total_distance > 10 and self.total_distance < 20 and self.demo_state == 5:
                            if self.vehicle_state[2] > 0.1:
                                self.mission_state_msg.mission_state = MissionState.AS_EMERGENCY_BRAKE
                                # this might need to be handled by a message to safety node
                                self._publisher_mission_state.publish(self.mission_state_msg)
                                self.call_ebs_service()
                                self.get_logger().info('Emergency brake applied, stopping the car...')
                            elif self.vehicle_state[2] < 0.1:
                                if self.demo_state == 5:
                                    self.mission_state_msg.mission_state = MissionState.AS_FINISHED
                                    self.can_reply = True
                                    self._publisher_can_complete.publish(self.can_reply)
                                    self._publisher_mission_state.publish(self.mission_state_msg)
                                    #reset the distance travelled
                                    self.initial_odom = self.odom
                                    self.demo_state = 6
                    elif self.demo_state == 6:
                        self.get_logger().info('Demonstration mission finished, resetting state machine...')
                        self.mission_state = MissionState.AS_FINISHED
                        self.mission_state_msg.mission_state = self.mission_state
                        self.can_reply = True
                        self._publisher_can_complete.publish(self.can_reply)
                        self._publisher_mission_state.publish(self.mission_state_msg)
                        # reset the initial odom and time
                        self.initial_odom = Odometry()
                        self.initial_time = None
                        self.demo_state = 0

        #acceleration mission
        elif self.mission_type == Mission.AMI_ACCELERATION:
            if self.mission_state == MissionState.AS_READY:
                self.get_logger().info('Ready for Acceleration Mission...')
                self.initial_time = self.get_clock().now()
                self.initial_distance = self.distance
            elif self.mission_state == MissionState.AS_DRIVING:
                self.get_logger().info('AMI_Acceleration is ready, starting driving...')
                elapsed_time = (self.get_clock().now() - self.initial_time).nanoseconds / 1e9
                self.get_logger().info('Time = %.2f seconds' % elapsed_time)
                self.total_distance = self.distance - self.initial_distance
                self.get_logger().info('Total distance travelled: %.2f m' % self.total_distance)
                if self.total_distance <= 75:
                    self.mission_state_msg.mission_state = MissionState.AS_DRIVING
                    self._publisher_mission_state.publish(self.mission_state_msg)
                elif self.total_distance > 75:
                    if self.vehicle_state[2] > 0.1:
                        self.mission_state_msg.mission_state = MissionState.AS_BRAKE
                        self._publisher_mission_state.publish(self.mission_state_msg)
                    elif self.vehicle_state[2] < 0.1:
                            self.mission_state_msg.mission_state = MissionState.AS_FINISHED
                            self._publisher_mission_state.publish(self.mission_state_msg)
                            self.can_reply = True
                            self._publisher_can_complete.publish(self.can_reply)
            elif self.mission_state == MissionState.AS_FINISHED:
                self.get_logger().info('Mission Finished')

        # Skidpad mission
        elif self.mission_type == Mission.AMI_SKIDPAD:
            # figure of 8
            # go on go RES signal
            # 2 laps around each circle then stop
            # stop within 25m inside the straight section
            # when stopped transmit the AS_FINISHED state
            if self.mission_state == MissionState.AS_READY:
                self.get_logger().info('Ready for Skidpad Mission...')
            elif self.mission_state == MissionState.AS_DRIVING and self.drive_state == 0:
                self.get_logger().info('AMI_Skidpad is ready, starting driving...')
                self._publisher_mission_state.publish(self.mission_state_msg)
                self.drive_state = 1
        elif self.mission_type == Mission.AMI_AUTOCROSS:
            # go on goRES signal
            #1 lap around the autocross track
            # stop within 30m after the end of the first lap within the cones
            # when stopped transmit the AS_FINISHED state
            if self.mission_state == MissionState.AS_READY:
                self.get_logger().info('Ready for Skidpad Mission...')
            elif self.mission_state == MissionState.AS_DRIVING and self.drive_state == 0:
                self.get_logger().info('AMI_Skidpad is ready, starting driving...')
                self._publisher_mission_state.publish(self.mission_state_msg)
                self.drive_state = 1
        elif self.mission_type == Mission.AMI_TRACK_DRIVE:
            # go on goRES signal
            # 10 laps
            # stop within 30m after the end of the first lap within the cones
            # when stopped transmit the AS_FINISHED state
            if self.mission_state == MissionState.AS_READY:
                self.get_logger().info('Ready for Skidpad Mission...')
            elif self.mission_state == MissionState.AS_DRIVING and self.drive_state == 0:
                self.get_logger().info('AMI_Skidpad is ready, starting driving...')
                self._publisher_mission_state.publish(self.mission_state_msg)
                self.drive_state = 1

        # for static integration day testing only - no drive commands sent
        elif self.mission_type == Mission.AMI_ADS_INSPECTION:
            if self.mission_state == MissionState.AS_READY:
                self.get_logger().info('Starting inspection mission A...')
                self.initial_distance = self.distance
                self.initial_time = self.get_clock().now()
            if self.mission_state == MissionState.AS_DRIVING:
                self.get_logger().info('AMI_DDT_INSPECTION_A is ready, starting driving...')
                self.get_logger().info('Time = %s' % (self.get_clock().now() - self.initial_time))
                # For 0-10 s drive full steam ahead until 200rpm reached, for >10s apply brake stop
                if (self.get_clock().now() - self.initial_time) > Duration(seconds=10):
                    self.can_reply = True
                    self._publisher_can_complete.publish(self.can_reply)
                    self.get_logger().info('Telling can mission complete...')
            if self.mission_state == MissionState.AS_FINISHED:
                self.get_logger().info('Mission Finished received from CAN')
            if self.mission_state == MissionState.AS_EMERGENCY_BRAKE:
                self.get_logger().info('EMERGENCY BRAKE received from CAN')

        # for static integration day testing only - no drive commands sent
        elif self.mission_type == Mission.AMI_ADS_EBS:
            if self.mission_state == MissionState.AS_READY:
                self.get_logger().info('Starting inspection mission A...')
                self.initial_distance = self.distance
                self.initial_time = self.get_clock().now()
            elif self.mission_state == MissionState.AS_DRIVING:
                self.get_logger().info('AMI_DDT_INSPECTION_A is ready, starting driving...')
                self.get_logger().info('Time = %s' % (self.get_clock().now() - self.initial_time))
                # For 0-10 sim drive then call ebs
                if (self.get_clock().now() - self.initial_time) > Duration(seconds=10):
                    self.call_ebs_service()
                    self.get_logger().info('Emergency brake applied, stopping the car...')
                    self.get_logger().info('Telling can to deploy ebs...')
            elif self.mission_state == MissionState.AS_FINISHED:
                self.get_logger().info('Mission Finished received from CAN')
            elif self.mission_state == MissionState.AS_EMERGENCY_BRAKE:
                self.get_logger().info('EMERGENCY BRAKE received from CAN')
        elif self.mission_type == Mission.AMI_JOYSTICK:
            pass
        elif self.mission_type == Mission.AMI_MANUAL:
            pass
        elif self.mission_type == Mission.AMI_NEW_MISSION:
            pass
        else:
            self.get_logger().warn('Unknown mission selected: %s' % self.mission_type)



def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Mission_Control()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()