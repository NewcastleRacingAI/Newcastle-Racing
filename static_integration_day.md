### CAN Comms test
ros2 launch ros_can ros_can.launch.py

### Mission Control
ros2 launch ros_can ros_can.launch.py
ros2 run newcastle_racing_ai mission_control

AMI_ADS_INSPECTION
# Set mission and ready
ros2 topic pub /ros_can/state eufs_msgs/msg/CanState "{as_state: 1, ami_state: 16}"
ros2 topic pub /ros_can/state eufs_msgs/msg/CanState "{as_state: 2, ami_state: 16}"

should change to mission completed from can

AMI_ADS_EBS
# Set mission and ready
ros2 topic pub /ros_can/state eufs_msgs/msg/CanState "{as_state: 1, ami_state: 17}"
ros2 topic pub /ros_can/state eufs_msgs/msg/CanState "{as_state: 2, ami_state: 17}"


Demo
# Set mission and ready
ros2 topic pub /ros_can/state eufs_msgs/msg/CanState "{as_state: 1, ami_state: 18}"
ros2 topic pub /ros_can/state eufs_msgs/msg/CanState "{as_state: 2, ami_state: 18}"

should change to mission completed from can





