from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

PACKAGE_NAME = "newcastle_racing_ai"
NAMESPACE = "nrfai"

def generate_launch_description():
    workspace_dir = os.getenv('WORKSPACE_DIR', '/workspace')

    return LaunchDescription([
        # ros2 launch eufs_launcher eufs_launcher.launch.py
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(workspace_dir, 'eufs_sim/eufs_launcher/launch/eufs_launcher.launch.py')
        #     )
        # ),
        # ros_can launcher
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(workspace_dir, 'ros_can/launch/ros_can.launch.py')
        #     )
        #),
        # Path planning node (as a process)
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, 'install/ft-fsd-path-planning/bin/path_planning_node')],
            output='screen'
        ),
        # MPCC control node (as a ROS 2 node)
        # Node(
        #     package='mpcc_control',
        #     executable='mpcc_control_node',
        #     output='screen'
        # ),
        Node(
        package=PACKAGE_NAME,
        namespace=NAMESPACE,
        executable="controller",
        name="Controller",
        ),
        Node(
        package=PACKAGE_NAME,
        namespace=NAMESPACE,
        executable="safety",
        name="Safety",
        ),
        Node(
        package=PACKAGE_NAME,
        namespace=NAMESPACE,
        executable="mission_control",
        name="Mission_Control",
        ),
        Node(
        package=PACKAGE_NAME,
        namespace=NAMESPACE,
        executable="odometry",
        name="Odometry",
        ),
        # Newcastle Racing AI launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(workspace_dir, 'newcastle_racing_ai/launch/nra_launch.py')
        #     )
        # ),
        #launch ros_can
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(workspace_dir, 'ros_can/launch/ros_can.launch.py')
        #     )
        #)
    ])