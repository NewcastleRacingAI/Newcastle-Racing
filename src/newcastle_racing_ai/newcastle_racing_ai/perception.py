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


class Perception(Node):

    def __init__(self):
        super().__init__("Perception")
        self._img_counter = 0
        self._latest_image_msg = None
        self.camera_timer_period = self.get_parameter("camera_time_step").value
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        self._subscription = self.create_subscription(StereoImage, self.get_parameter("camera_topic").value, self._on_camera, 10)
        #self._subscription = self.create_subscription(Imu, self.get_parameter("imu_topic").value, self._on_imu, 10)
        #self._subscription = self.create_subscription(PointCloud2, self.get_parameter("lidar_topic").value, self._on_lidar, 10)
        self._publisher = self.create_publisher(ConeArrayWithCovariance, self.get_parameter("cones_topic").value, 10)
        self.timer = self.create_timer(self.camera_timer_period, self._timer_callback)

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
        self.cone_model = YOLO('runs/detect/train6/weights/best.pt')

    def _on_camera(self, msg):
        self.get_logger().info('Received: "%s"' % type(msg))
        # Process CAMERA data here if needed. For example, saving the image in ppm format.
        self._latest_image_msg = msg
        
    def _on_imu(self, msg):
        self.get_logger().info('Received: "%s"' % type(msg))
        # Process IMU data here if needed

    def _timer_callback(self):
        self.save_image_ppm(msg)
        # Code here to process the latest image message and publish cones.

        # msg = ConeArrayWithCovariance(
        #     blue_cones=[ConeWithCovariance(point=Point(x=0.0, y=0.0, z=0.0), covariance=[0.0, 0.0, 0.0, 0.0])],
        #     yellow_cones=[],
        #     orange_cones=[],
        #     big_orange_cones=[],
        #     unknown_color_cones=[ConeWithCovariance(point=Point(x=0.0, y=0.0, z=0.0), covariance=[0.0, 0.0, 0.0, 0.0])],
        # )
        # self._publisher.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg)

    def run_camera(self):
        while True:
            if self.zed.grab(self.runtime_param) == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
                self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
                self.zed.retrieve_measure(self.depth_measure, sl.MEASURE.DEPTH)
                cones_df = pd.DataFrame(columns=['ConeColour', 'X', 'Y', 'Z'])
                image_np = self.image.get_data()
                image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

                # Run cone prediction on image
                results = self.cone_model([image_np])[0]

                # Get coords of each cone in image
                for cone in results.boxes:
                    x,y,w,h = cone.xywh[0]
                    x = round(float(x))
                    y = round(float(y))
                    
                    # Get the 3D point cloud values for pixel (i, j)
                    cone_x, cone_y, cone_z, _ = self.point_cloud.get_value(x, y)[1]
                    cone_class = self.cone_model.names[int(cone.cls)][:-5]

                    # Check if cone already exists
                    buffer = 0.2
                    if len(cones_df[
                        (cones_df['ConeColour'] == cone_class) &
                        (cones_df['X'].between(cone_x-buffer, cone_x+buffer)) &
                        (cones_df['Z'].between(cone_z-buffer, cone_z+buffer))
                        ]) == 0:
                        cones_df.loc[len(cones_df)] = {'ConeColour': cone_class, 'X': cone_x, 'Y': 0, 'Z': cone_z}
                i = i+1

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
