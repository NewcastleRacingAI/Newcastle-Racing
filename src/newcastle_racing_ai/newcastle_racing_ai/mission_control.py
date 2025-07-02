import rclpy
from rclpy.node import Node
from eufs_msgs.msg import  CanState
from std_msgs.msd import Bool
from newcastle_racing_ai_msgs import Mission, MissionState
from .parameters import PARAMETERS

class Mission_Control(Node):
    """
    This node is listening for the go signal and mission selection from the ros_can node, which it passes on to the controller and allows the car to start driving.
    A separate node listens for any safety/ebs related signals. 
    """

    def __init__(self):
        super().__init__('Mission_Control')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        self._subscription = self.create_subscription(CanState, self.get_parameter("can_state_topic").value, self._on_state, 10)
        self._publisher_mission = self.create_publisher(Mission, self.get_parameter("mission_topic").value, 10)
        self._publisher_mission_state = self.create_publisher(MissionState, self.get_parameter("mission_state_topic").value, 10)
        self._timer = self.create_timer(0.1, self._publish_ebs)  # 10 Hz
        self.mission = Mission.AS_OFF
        self.mission = MissionState.AMI_NOT_SELECTED


    def _on_state(self, msg):
        self.get_logger().info('Received: "%s"' % type(msg))
        if msg.as_state == AS_DRIVING or msg.as_state == AS_FINISHED:
            self.mission = msg.as_state
            self._publisher_mission.publish(self.mission)
        else:
            #do nothing
            pass
        self.mission = msg.ami_state
        self._publisher_mission_state.publish(self.mission)
        
    

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Planner()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()