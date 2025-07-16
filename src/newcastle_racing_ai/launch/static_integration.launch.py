from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
import os

PACKAGE_NAME = "newcastle_racing_ai"
NAMESPACE = "nrfai"

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_can"), "launch", "ros_can.launch.py")
            ),
        ),
        Node(
        package=PACKAGE_NAME,
        namespace=NAMESPACE,
        executable="mission_control",
        name="Mission_Control",
        ),
    ])