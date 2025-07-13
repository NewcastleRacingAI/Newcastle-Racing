import rclpy
from rclpy.node import Node
import numpy as np
from eufs_msgs.msg import ConeArrayWithCovariance, CarState,PathWithBoundaries
from matplotlib import pyplot as plt
from sensor_msgs.msg import Imu
from fsd_path_planning import PathPlanner, MissionTypes
from math import atan2, radians
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point
from newcastle_racing_ai_msgs.msg import MissionState, Mission, NewCarState

class PathPlanningDualCamVisualizationNode(Node):
    def __init__(self):
        super().__init__('path_planning_dual_cam_visualization_node')
        self.mission_received = False

        # 初始化小车状态与路径
        self.global_cones = [np.zeros((0, 2)) for _ in range(5)]
        self.path = ()
        self.car_position = np.array([0.0, 0.0])
        self.car_direction = np.array([1.0, 0.0])

        # 控制参数
        self.k_gain = 0.3
        self.look_ahead_distance = 3.0
        self.max_speed = 0.5

        # 初始化发布器与定时器
        #self.create_subscription(CarState, '/odometry_integration/car_state', self.car_state_callback, 10)
        self.create_subscription(NewCarState, '/car_state', self.car_state_callback, 10)
        self.create_subscription(ConeArrayWithCovariance, '/ground_truth/cones', self.cones_callback, 10)
        #self.create_subscription(Imu, '/camera/imu/data', self.imu_callback, 10)
        self.create_subscription(Mission, '/mission', self._on_mission, 10)
        self.path_publisher = self.create_publisher(PathWithBoundaries, '/path', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 1)
        self.create_timer(0.05, self.publish_path)
        # self.create_timer(1, self.stanley_control)

        # 初始化 Matplotlib 图形
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-30, 30)
        self.ax.set_ylim(-30, 30)
        plt.ion()
        plt.show()

    def _on_mission(self, msg):
        """
        Callback to handle mission messages.
        This sets the mission type for the path planner.
        """
        self.mission_received = True
        self.get_logger().info(f"Received mission: {msg.mission}")
        if msg.mission == Mission.AMI_ACCELERATION:
            self.planner = PathPlanner(MissionTypes.acceleration)
            self.get_logger().info("Mission set to AMI_ACCELERATION")
        elif msg.mission == Mission.AMI_SKIDPAD:
            self.planner = PathPlanner(MissionTypes.skidpad)
            self.get_logger().info("Mission set to AMI_SKIDPAD")
        elif msg.mission == Mission.AMI_AUTOCROSS:
            self.planner = PathPlanner(MissionTypes.autocross)
            self.get_logger().info("Mission set to AMI_AUTOCROSS")
        elif msg.mission == Mission.AMI_TRACK_DRIVE:
            self.planner = PathPlanner(MissionTypes.trackdrive)
            self.get_logger().info("Mission set to AMI_TRACK_DRIVE")
        elif msg.mission == Mission.AMI_ADS_EBS:
            self.planner = PathPlanner(MissionTypes.ebs_test)
            self.get_logger().info("Mission set to AMI_EBS_TEST")
        elif msg.mission in (Mission.AMI_AUTONOMOUS_DEMO, Mission.AMI_DDT_INSPECTION_A, Mission.AMI_DDT_INSPECTION_B, Mission.AMI_ADS_DEMO, Mission.AMI_ADS_INSPECTION):
            self.planner = PathPlanner(MissionTypes.inspection)
            self.get_logger().info("Mission set to AMI_INSPECTION")
        elif msg.mission == Mission.AMI_MANUAL \
            or self.mission == Mission.AMI_JOYSTICK:
            self.planner = PathPlanner(MissionTypes.manual_driving)
            self.get_logger().info("Mission set to AMI_MANUAL_DRIVING")
        else:
            # this should just catch AMI_NEW_MISSION and AMI_NOT_SELECTED
            self.get_logger().info("Mission not selected, waiting for mission type.")
            self.planner = PathPlanner(MissionTypes.none)
            self.mission_received = False
    
    

    def publish_path(self):
        """发布路径的可视化数据"""
        if (
            not isinstance(self.path, tuple)
            or len(self.path) == 0
            or len(self.path[0]) == 0
            or self.mission_received == False
        ):
            return

        path = self.path[0]
        marker = Marker()
        marker.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="base_footprint")
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        for point in path:
            x_local = point[1] - self.car_position[0]
            y_local = point[2] - self.car_position[1]
            marker.points.append(Point(x=x_local, y=y_local, z=0.0))

        self.marker_pub.publish(marker)


    # def imu_callback(self, imu_msg):
    #     _, _, yaw = quaternion_to_euler(imu_msg.orientation)
    #     self.car_direction = np.array([np.cos(yaw), np.sin(yaw)])
    #     self.check_and_compute_path()

    def car_state_callback(self, msg):
        # position = msg.pose.pose.position
        # self.car_position = np.array([position.x, position.y])
        self.car_position = np.array([msg.x, msg.y])
        self.car_direction = np.array([np.cos(msg.yaw), np.sin(msg.yaw)])
        self.check_and_compute_path()

    def cones_callback(self, msg):
        cone_coords = {'UNKNOWN': [], 'RIGHT': [], 'LEFT': [], 'START_FINISH_AREA': [], 'START_FINISH_LINE': []}
        cone_types = {'unknown_color_cones': 'UNKNOWN', 'yellow_cones': 'RIGHT', 'blue_cones': 'LEFT', 'orange_cones': 'START_FINISH_AREA', 'big_orange_cones': 'START_FINISH_LINE'}

        for cone_type, enum_type in cone_types.items():
            for cone in getattr(msg, cone_type, []):
                relative_position = [cone.point.x + self.car_position[0], cone.point.y + self.car_position[1]]
                cone_coords[enum_type].append(relative_position)

        self.global_cones = [np.array(cone_coords['UNKNOWN']), np.array(cone_coords['RIGHT']), np.array(cone_coords['LEFT']), np.array(cone_coords['START_FINISH_AREA']), np.array(cone_coords['START_FINISH_LINE'])]
        self.check_and_compute_path()

    def check_and_compute_path(self):
        if self.mission_received:
            out = self.planner.calculate_path_in_global_frame(
            self.global_cones, self.car_position, self.car_direction, return_intermediate_results=True
            )
            (path, sorted_left, sorted_right, left_cones_with_virtual, right_cones_with_virtual, left_to_right_match, right_to_left_match) = out

            # 确保将所有参数传递给 `publish_path_with_boundaries`
            self.publish_path_with_boundaries(
                path, sorted_left, sorted_right, left_cones_with_virtual, right_cones_with_virtual, left_to_right_match, right_to_left_match
            )
            self.path = out
            self.update_plot()
        else:
            self.get_logger().info("Mission not received yet, skipping path computation.")
    

    def publish_path_with_boundaries(self, path, sorted_left, sorted_right, left_cones_with_virtual, right_cones_with_virtual, left_to_right_match, right_to_left_match):
        """发布路径和边界的自定义消息"""
        path_msg = PathWithBoundaries()
        path_msg.path = [Point(x=p[1], y=p[2], z=0.0) for p in path]
        path_msg.sorted_left = [Point(x=p[0], y=p[1], z=0.0) for p in sorted_left]
        path_msg.sorted_right = [Point(x=p[0], y=p[1], z=0.0) for p in sorted_right]
        path_msg.left_cones_with_virtual = [Point(x=p[0], y=p[1], z=0.0) for p in left_cones_with_virtual]
        path_msg.right_cones_with_virtual = [Point(x=p[0], y=p[1], z=0.0) for p in right_cones_with_virtual]
       # 检查匹配列表
        path_msg.left_to_right_match = [int(val) for val in left_to_right_match if isinstance(val, int) and -2147483648 <= val <= 2147483647]
        path_msg.right_to_left_match = [int(val) for val in right_to_left_match if isinstance(val, int) and -2147483648 <= val <= 2147483647]

        self.path_publisher.publish(path_msg)

    def update_plot(self):
        try:
            if not isinstance(self.path, tuple) or len(self.path[0]) == 0:
                return

            path, sorted_left, sorted_right = self.path[0], self.path[1], self.path[2]

            self.ax.cla()
            self.ax.set_title("Computed Path and Cones")
            self.ax.set_xlabel("X Coordinate")
            self.ax.set_ylabel("Y Coordinate")
            self.ax.axis("equal")

            if sorted_left.size > 0:
                self.ax.scatter(sorted_left[:, 0], sorted_left[:, 1], c='blue', label='Left Cones')
            if sorted_right.size > 0:
                self.ax.scatter(sorted_right[:, 0], sorted_right[:, 1], c='yellow', label='Right Cones')

            if path.size > 0:
                self.ax.plot(path[:, 1], path[:, 2], 'r-', label='Path')

            self.ax.legend()
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

        except Exception as e:
            print(f"Error updating plot: {e}")

def quaternion_to_euler(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = atan2(t0, t1)
    t2 = 2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = np.arcsin(t2)
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)
    return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningDualCamVisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
