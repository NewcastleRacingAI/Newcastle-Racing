#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class CameraCaptureNode(Node):


    def __init__(self):
        super().__init__('capture_camera_node')
        self.subscription = self.create_subscription(
            Image,
            '/zed/image_raw',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        

    def listener_callback(self, msg):
        # current_dir = os.path.dirname(os.path.realpath(__file__))

        # # DEBUG ~
        # #~~~~~~~~
        # self.get_logger().info(f'Current Path of file:\n{current_dir}')

        # # Define the relative path to the artifacts directory
        # artifacts_dir = os.path.join(current_dir, '..', 'artifacts')
        
        # # Paths to your specific files
        # weights_path = os.path.join(artifacts_dir, 'pretrained_yolo.weights')
        # cfg_path = os.path.join(artifacts_dir, 'yolo_baseline.cfg')

        # # DEBUG ~
        # #~~~~~~~~
        # self.get_logger().info(f'Model Architecture and Weights:\n{weights_path}\n{cfg_path}')

        # self.get_logger().info('Receiving video frame...')
        

        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Setup path to the MIT Repo
        current_dir = os.path.dirname(os.path.realpath(__file__))
        cvc_yolov3 = os.path.join(current_dir, '..', 'MIT-Driverless-CV-TrainingInfra', 'CVC-YOLOv3')
        
        import sys
        sys.path.append(cvc_yolov3)
        from detect import main_input
        
        # self.get_logger().info(f"Current Path : {os.getcwd()}")
        # self.get_logger().info('Starting CVC-Yolov3 On Video...')
        main_input(current_frame)
        self.get_logger().info('Retrieved Frame that ran on CVC-YOLOv3')

        # Display image
        # cv2.imshow("Camera Frame", current_frame)
        # cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    camera_capture_node = CameraCaptureNode()
    rclpy.spin(camera_capture_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_capture_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
