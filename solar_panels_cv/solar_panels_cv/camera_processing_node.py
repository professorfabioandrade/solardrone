import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2

from solar_panels_cv.utils.azimuth_detector import AzimuthDetector

class CameraProcessingNode(Node):
    def __init__(self) -> Node:
        super().__init__('camera_handler')
        
        _ = self.create_subscription(
            Image,
            '/airsim_node/Drone_1/camera_1/Scene',
            self.img_callback,
            10)
        
        self.tilt_angle_pub = self.create_publisher(Float32, '/tilt_angle', 10)

        self.br = CvBridge()

        self.azimuth = AzimuthDetector(self.get_logger().info)

    def img_callback(self, msg: Image) -> None:
        try:
            # Convert ROS Image message to a numpy array
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            np_image = np.array(cv_image)
            
            tilt_angle, distance = self.azimuth(np_image)
            self.pub_angle(tilt_angle, distance)

            if self.azimuth.edges is not None:
                cv2.imshow("Canny", self.azimuth.edges)
                cv2.waitKey(1)  # Add a delay to allow the image to be rendered
            
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')

        except Exception as e:
            self.get_logger().error(f'Something else happened: {e}')

    def pub_angle(self, angle: float, distance: float) -> None:
        if angle is not None:
            print(f"The tilt angle of the horizontal line closest to the middle is {angle:.2f} degrees at a distance of {distance:.2f}.")
            msg = Float32()
            msg.data = angle
            self.tilt_angle_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_processing_node = CameraProcessingNode()
    rclpy.spin(camera_processing_node)
    camera_processing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()