import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition

PACKAGE_NAME = "newcastle_racing_ai"
NAMESPACE = "nrfai"


def generate_launch_description():
    node_parameters = [
        {
            "camera_topic": LaunchConfiguration("camera_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "lidar_topic": LaunchConfiguration("lidar_topic"),
            "cones_topic": LaunchConfiguration("cones_topic"),
            "path_topic": LaunchConfiguration("path_topic"),
            "cmd_topic": LaunchConfiguration("cmd_topic"),
            "time_step": LaunchConfiguration("time_step"),
            "ebs_topic": LaunchConfiguration("ebs_topic"),
        }
    ]
    launch_bridge = LaunchConfiguration("launch_bridge")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_topic", default_value=TextSubstitution(text="camera")),
            DeclareLaunchArgument("imu_topic", default_value=TextSubstitution(text="imu")),
            DeclareLaunchArgument("lidar_topic", default_value=TextSubstitution(text="lidar")),
            DeclareLaunchArgument("cones_topic", default_value=TextSubstitution(text="cones")),
            DeclareLaunchArgument("path_topic", default_value=TextSubstitution(text="path")),
            DeclareLaunchArgument("cmd_topic", default_value=TextSubstitution(text="cmd")),
            DeclareLaunchArgument("time_step", default_value=TextSubstitution(text="0.0")),
            DeclareLaunchArgument("ebs_topic", default_value=TextSubstitution(text="0.0")),
            # DeclareLaunchArgument("launch_bridge", default_value="True"),
            # IncludeLaunchDescription(
            #     FrontendLaunchDescriptionSource(
            #         os.path.join(
            #             get_package_share_directory("rosbridge_server"), "launch", "rosbridge_websocket_launch.xml"
            #         )
            #     ),
            #     condition=IfCondition(launch_bridge),
            # ),
            Node(
                package=PACKAGE_NAME,
                namespace=NAMESPACE,
                executable="controller",
                name="controller",
                parameters=node_parameters,
            )
            # Node(
            #     package=PACKAGE_NAME,
            #     namespace=NAMESPACE,
            #     executable="perception",
            #     name="perception",
            #     parameters=node_parameters,
            # ),
            # Node(
            #     package=PACKAGE_NAME,
            #     namespace=NAMESPACE,
            #     executable="planner",
            #     name="planner",
            #     parameters=node_parameters,
            # ),
        ]
    )


