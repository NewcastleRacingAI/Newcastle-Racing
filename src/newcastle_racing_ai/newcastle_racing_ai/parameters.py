PARAMETERS = (
    # TOPICS
    ("camera_topic", "/stereo_camera"),
    ("cones_topic", "/cones"),
    ("path_topic", "/path"),
    ("cmd_topic", "/cmd"),
    ("imu_topic", "/camera/imu/data"),
    ("odom_topic", "nrfai/odom"),
    ("can_state_topic","/ros_can/state"),
    ("ebs_topic", "/ebs"),
    ("mission_topic", "/mission"),
    ("mission_state_topic", "/mission_state"),
    # CONFIGURATION
    ("time_step", 0.0),
    # determines the camera fps that the perception node will run at
    ("camera_time_step", 0.05),
)
