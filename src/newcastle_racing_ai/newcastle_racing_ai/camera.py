import os
import rclpy
from rclpy.node import Node
from eufs_mgs.msg import StereoImage
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Point
from newcastle_racing_ai_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance
from .parameters import PARAMETERS
import pyzed.sl as sl
import cv2
import pandas as pd
from ultralytics import YOLO


class Camera(Node):

    def __init__(self):
        super().__init__("camera_node")
        self.camera_timer_period = self.get_parameter("camera_time_step").value
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        self._subscription = self.create_subscription(Image, '/zed/left/image_rect_color', self._on_camera, 10)
        self._subscription = self.create_subscription(Image, '/zed/points', self._on_depth_camera, 10)
        self.camera_left_publisher = self.create_publisher(Image, self.get_parameter("camera_left_topic").value, 10)
        self.camera_depth_publisher = self.create_publisher(Image, self.get_parameter("camera_depth_topic").value, 10)
        self.timer = self.create_timer(self.camera_timer_period, self._timer_callback)
        self.sim_mode = True


        # Set up camera
        self.zed = sl.Camera()

        # Set configuration parameters
        init_params = sl.InitParameters()
        init_params.coordinate_units = sl.UNIT.METER
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(repr(err))
            exit(-1)

        self.runtime_param = sl.RuntimeParameters()
        self.image = sl.Mat()
        self.point_cloud = sl.Mat()
        self.depth_measure = sl.Mat()

    
    def _on_depth_camera(self, msg):
        # may need to treat data 
        self.get_logger().info('Received: "%s"' % type(msg))
        self.point_cloud = msg
        self.camera_depth_publisher.publish(msg)

    def _on_camera(self, msg):
        # may need to treat data
        self.get_logger().info('Received: "%s"' % type(msg))
        self.image = msg
        self.camera_left_publisher.publish(msg)


    def _timer_callback(self):
        if self.sim_mode:
            self.save_image_ppm(self.self.image)
            self.save_image_ppm(self.self.point_cloud)
        else:
            self.grab_frame()
            self.camera_left_publisher.publish(self.image)
            self.camera_depth_publisher.publish(self.point_cloud)

    def grab_frame(self):
        if self.zed.grab(self.runtime_param) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
            self.zed.retrieve_measure(self.depth_measure, sl.MEASURE.DEPTH)
                

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
    minimal_publisher = Camera()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
