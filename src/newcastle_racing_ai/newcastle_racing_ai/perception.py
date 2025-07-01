import os
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, Imu, PointCloud2
from geometry_msgs.msg import Point
from newcastle_racing_ai_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance
from .parameters import PARAMETERS


class Perception(Node):

    def __init__(self):
        super().__init__("Perception")
        self._img_counter = 0
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        self._subscription = self.create_subscription(
            Image, self.get_parameter("camera_topic").value, self._on_camera, 10
        )
        self._subscription = self.create_subscription(Imu, self.get_parameter("imu_topic").value, self._on_imu, 10)
        self._subscription = self.create_subscription(
            PointCloud2, self.get_parameter("lidar_topic").value, self._on_lidar, 10
        )
        self._publisher = self.create_publisher(ConeArrayWithCovariance, self.get_parameter("cones_topic").value, 10)
        # self.timer = self.create_timer(timer_period, self._timer_callback)

    def _on_camera(self, msg):
        self.get_logger().info('Received: "%s"' % type(msg))
        # Process CAMERA data here if needed. For example, saving the image in ppm format.
        self.save_image_ppm(msg)

    def _on_imu(self, msg):
        self.get_logger().info('Received: "%s"' % type(msg))
        # Process IMU data here if needed

    def _on_lidar(self, msg):
        self.get_logger().info('Received: "%s"' % type(msg))
        # Process LIDAR data here if needed

    def _timer_callback(self):
        msg = ConeArrayWithCovariance(
            blue_cones=[ConeWithCovariance(point=Point(x=0.0, y=0.0, z=0.0), covariance=[0.0, 0.0, 0.0, 0.0])],
            yellow_cones=[],
            orange_cones=[],
            big_orange_cones=[],
            unknown_color_cones=[ConeWithCovariance(point=Point(x=0.0, y=0.0, z=0.0), covariance=[0.0, 0.0, 0.0, 0.0])],
        )
        self._publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

    def save_image_ppm(self, msg, filename="/workspace/newcastle_racing_ai/imgs/image-{}.ppm"):
        # This function is a placeholder for saving the image in ppm format.
        self.get_logger().info("Saving image in ppm format.")
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warn("Can only handle rgb8 or bgr8 encoding.")
            return

        # Check that the path to the image directory exists
        if not os.path.exists(os.path.dirname(filename.format(self._img_counter))):
            os.makedirs(os.path.dirname(filename.format(self._img_counter)))

        self._img_counter += 1
        with open(filename.format(self._img_counter), "w", encoding="utf-8") as file:
            file.write("P3\n")
            file.write(f"{msg.width} {msg.height}\n")
            file.write("255\n")

            for y in range(msg.height):
                for x in range(msg.width):
                    # Get indices for the pixel components
                    first_byte_idx = y * msg.step + 3 * x
                    green_byte_idx = first_byte_idx + 1
                    last_byte_idx = first_byte_idx + 2

                    if msg.encoding == "rgb8":
                        red_byte_idx = first_byte_idx
                        blue_byte_idx = last_byte_idx
                    elif msg.encoding == "bgr8":
                        red_byte_idx = last_byte_idx
                        blue_byte_idx = first_byte_idx
                    else:
                        self.get_logger().warn("Can only handle rgb8 or bgr8 encoding.")
                        return

                    file.write(f"{msg.data[red_byte_idx]} {msg.data[green_byte_idx]} {msg.data[blue_byte_idx]} ")
                file.write("\n")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Perception()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
