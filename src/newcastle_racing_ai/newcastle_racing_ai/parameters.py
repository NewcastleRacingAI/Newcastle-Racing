PARAMETERS = (
    # TOPICS
    ("camera_left_topic", "/camera/left"),
    ("camera_depth_topic", "/camera/depth"),
    ("cones_topic", "/cones"),
    ("path_topic", "/path"),
    ("cmd_topic", "/cmd"),
    ("imu_topic", "/imu/data"),
    ("odom_topic", "/odom"),#live = /odom,
    ("gt_car_state_topic", "/odometry_integration/car_state"),
    ("car_state_topic", "/car_state"),
    ("can_state_topic","/ros_can/state"), # from ros_can
    ("can_wheel_speed_topic", "/ros_can/wheel_speeds"), # from ros_can
    ("ebs_topic", "/ebs"), # our internal ebs topic
    ("can_driving_flag_topic", "/state_machine/driving_flag"), # from ros_can
    ("can_ebs_request_service", "/ros_can/ebs"), # for an internal ebs request to the CAN node
    ("can_mission_complete_topic", "/ros_can/mission_completed"), # for an internal mission complete topic to the CAN node
    ("mission_topic", "/mission"), # our internal mission topic
    ("mission_state_topic", "/mission_state"), # our internal mission state topic
    # CONFIGURATION
    ("time_step", 0.0),
    # determines the camera fps that the perception node will run at
    ("camera_time_step", 1),
)

