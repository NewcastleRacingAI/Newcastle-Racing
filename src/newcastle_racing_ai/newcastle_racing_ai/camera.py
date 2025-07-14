import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from .parameters import PARAMETERS
import pandas as pd
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from PIL import Image as PILImage
import numpy as np
import pyzed.sl as sl



class Camera(Node):

    def __init__(self):
        super().__init__("camera_node")
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        self.camera_timer_period = self.get_parameter("camera_time_step").value
        image_qos = QoSProfile(depth=10)
        image_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self._subscription = self.create_subscription(Image, '/zed/left/image_rect_color', self._on_camera, image_qos)
        self._subscription = self.create_subscription(Image, '/zed/depth/image_raw', self._on_depth_camera, image_qos)
        self.camera_left_publisher = self.create_publisher(Image, self.get_parameter("camera_left_topic").value, 10)
        self.camera_depth_publisher = self.create_publisher(Image, self.get_parameter("camera_depth_topic").value, 10)
        self.timer = self.create_timer(self.camera_timer_period, self._timer_callback)
        self.sim_mode = True
        self.image = Image()
        self.depth = Image()
        self._img_counter = 0
        self.root = "/home/newcastleracing/Projects/Newcastle-Racing/src"

        # If using a real camera, uncomment the following lines to initialize the ZED camera
        #Set up camera
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
        self.depth = msg
        self.camera_depth_publisher.publish(msg)

    def _on_camera(self, msg):
        # may need to treat data
        self.get_logger().info('Received: "%s"' % type(msg))
        self.image = msg
        self.camera_left_publisher.publish(msg)


    def _timer_callback(self):
        if self.sim_mode:
            self.save_image_jpeg(self.image, type="left")
            self.save_image_jpeg(self.depth, type="depth")
        else:
            self.grab_frame()
            self.camera_left_publisher.publish(self.image)
            self.camera_depth_publisher.publish(self.depth)

    def grab_frame(self):
        if self.zed.grab(self.runtime_param) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            self.zed.retrieve_image(self.depth, sl.VIEW.DEPTH)
            #self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
            #self.zed.retrieve_measure(self.depth_measure, sl.MEASURE.DEPTH)
                

    def save_image_jpeg(self, msg, type, filename=None):
        # Save both color and depth images as JPEG (depth will be normalized to 8-bit)
        if filename is None:
            filename = os.path.join(self.root, "/newcastle_racing_ai/imgs/image_{}_{}.jpg")
        self.get_logger().info("Saving image in JPEG format.")
        # Check that the path to the image directory exists
        if not os.path.exists(os.path.dirname(filename.format(self._img_counter, type))):
            os.makedirs(os.path.dirname(filename.format(self._img_counter, type)))

        self._img_counter += 1

        if msg.encoding in ("rgb8", "bgr8"):
            # Convert ROS Image to numpy array
            img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            if msg.encoding == "bgr8":
                img_np = img_np[..., ::-1]  # Convert BGR to RGB
            pil_img = PILImage.fromarray(img_np, mode="RGB")
            pil_img.save(filename.format(self._img_counter, type), "JPEG")
        elif msg.encoding == "32FC1":
            # Depth image: convert float32 to 8-bit grayscale for visualization
            depth_np = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            # Normalize depth to 0-255 for visualization (clip to 0-10m for example)
            depth_norm = np.clip(depth_np, 0, 10)
            depth_norm = (depth_norm / 10.0 * 255).astype(np.uint8)
            pil_img = PILImage.fromarray(depth_norm, mode="L")
            pil_img.save(filename.format(self._img_counter, type), "JPEG")
        else:
            self.get_logger().warn(f"Cannot save image with encoding: {msg.encoding}")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Camera()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
