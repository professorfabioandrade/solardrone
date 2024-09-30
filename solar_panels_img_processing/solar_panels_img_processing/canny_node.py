from typing import Tuple
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from geometry_msgs.msg import Polygon, Point32
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2

from solar_panels_img_processing.utils.azimuth_detector import AzimuthDetector

class CannyProcessingNode(Node):
    def __init__(self, show_lines: bool = False) -> Node:
        super().__init__('canny_node')

        self.process_parameters()
        self.initialize_publishers()
        self.initialize_subscribers()
                
        self.br = CvBridge()
        self.azimuth = AzimuthDetector(self.get_logger().info, vertical=self.vertical_lines)
        self.show_lines = show_lines
        self.frame = None

    # ------ Initializing the components ------ #

    def initialize_publishers(self) -> None:
        self.tilt_angle_pub = self.create_publisher(Polygon, self.canny_line_topic_name, 10)

    def initialize_subscribers(self) -> None:
        _ = self.create_subscription(
            Image,
            self.airsim_imgs_topic_name,
            self.img_callback,
            10)
        
        _ = self.create_subscription(Empty, self.camera_trigger_topic_name, self.trigger_callback, 10)

    # ------ Processing Parameters ------ #

    def process_parameters(self) -> None:
        self.declare_parameter("vertical_lines", rclpy.Parameter.Type.BOOL)

        self.declare_parameter("airsim_imgs_topic_name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("camera_trigger_topic_name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("canny_line_topic_name", rclpy.Parameter.Type.STRING)

        self.vertical_lines = self.get_parameter("vertical_lines").get_parameter_value().bool_value

        self.airsim_imgs_topic_name = self.get_parameter("airsim_imgs_topic_name").get_parameter_value().string_value
        self.camera_trigger_topic_name = self.get_parameter("camera_trigger_topic_name").get_parameter_value().string_value
        self.canny_line_topic_name = self.get_parameter("canny_line_topic_name").get_parameter_value().string_value
            
    # ------ Handling Callbacks ------ #

    def img_callback(self, msg: Image) -> None:
        self.frame = msg

    def trigger_callback(self, _: Empty) -> None:
        try:
            # Convert ROS Image message to a numpy array
            cv_image = self.br.imgmsg_to_cv2(self.frame, desired_encoding='bgr8')
            np_image = np.array(cv_image)
            
            tilt_angle, distance, pos = self.azimuth(np_image)
            self.pub_angle(tilt_angle, distance, pos)
            if self.azimuth.edges is not None:
                if self.show_lines:
                    if self.azimuth.lines is not None:
                        for line in self.azimuth.lines:
                            x1, y1, x2, y2 = line[0]
                            cv2.line(np_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                else:
                    if pos:
                        x1, y1, x2, y2 = pos
                        print(pos)
                        cv2.line(np_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                                
                cv2.imshow("Lines", np_image)
                cv2.imshow("Canny", self.azimuth.edges)
                cv2.waitKey(1)
            
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')

        except Exception as e:
            self.get_logger().error(f'Something else happened: {e}')

    # ------ Publisher function ------ #

    def pub_angle(self, angle: float, distance: float, pos: Tuple[int, int, int, int]) -> None:
        if angle is not None:
            self.get_logger().info(f"The tilt angle of the horizontal line closest to the middle is {angle:.2f} degrees at a distance of {distance:.2f}.")
            msg = Polygon()
            start = Point32()
            end = Point32()
            start.x, start.y = float(pos[0]), float(pos[1])
            msg.points.append(start)
            end.x, end.y = float(pos[2]), float(pos[3])
            msg.points.append(end)
            self.tilt_angle_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_processing_node = CannyProcessingNode()
    rclpy.spin(camera_processing_node)
    camera_processing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()