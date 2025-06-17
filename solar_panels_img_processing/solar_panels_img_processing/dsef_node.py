from typing import Tuple, List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from geometry_msgs.msg import Polygon, Point32
from cv_bridge import CvBridge, CvBridgeError
import cv2

from solar_panels_img_processing.utils.dsef3_handler import DSEF3Handler
from solar_panels_img_processing.utils.csv_handler import CSVHandler
from time import time
import traceback 


class DSEFNode(Node):
    def __init__(self) -> Node:
        super().__init__('dsef_node')
        
        self.process_parameters()
        self.initialize_publishers()
        self.initialize_subscribers()

        self.br = CvBridge()
        initial_direction_deg = 270
        dir_span = 60
        
        #start_pix = (0+200, 0+200)
        start_pix = (200, 1080/4)
        end_pix = (1920,3*1080/4)
        speed = "high"
        debug = False
        self.dsef = DSEF3Handler(initial_direction_deg, dir_span, start_pix, end_pix, speed, debug)
        #self.dsef = DSEFHandler(3*math.pi/2)
        self.frame = None

        self.old_points = None

    # ------ Initializing the components ------ #

    def initialize_publishers(self) -> None:
        self.tilt_angle_pub = self.create_publisher(Polygon, self.dsef_line_topic_name, 10)

    def initialize_subscribers(self) -> None:
        _ = self.create_subscription(
            Image,
            self.airsim_imgs_topic_name,
            self.img_callback,
            10)
        
        _ = self.create_subscription(Empty, self.camera_trigger_topic_name, self.trigger_callback, 10)

    # ------ Processing Parameters ------ #

    def process_parameters(self) -> None:
        self.declare_parameter("vertical_lines", False)

        self.declare_parameter("airsim_imgs_topic_name", "/airsim_node/Drone_1/camera_1/Scene")
        self.declare_parameter("camera_trigger_topic_name", "/camera_trigger")
        self.declare_parameter("dsef_line_topic_name", "/detected_line")

        self.vertical_lines = self.get_parameter("vertical_lines").get_parameter_value().bool_value

        self.airsim_imgs_topic_name = self.get_parameter("airsim_imgs_topic_name").get_parameter_value().string_value
        self.camera_trigger_topic_name = self.get_parameter("camera_trigger_topic_name").get_parameter_value().string_value
        self.dsef_line_topic_name = self.get_parameter("dsef_line_topic_name").get_parameter_value().string_value

    # ------ Handling Callbacks ------ #

    def img_callback(self, msg: Image) -> None:
        self.frame = msg

    def trigger_callback(self, _: Empty) -> None:
        try:
            # Convert ROS Image message to a numpy array
            if self.frame is not None:
                cv_image = self.br.imgmsg_to_cv2(self.frame, desired_encoding='bgr8')
                start = time()
                points = self.dsef(cv_image)
                end = time()
                if points:
                    self.old_points = points
                    self.pub_points(points)
                else:
                    if self.old_points is not None:
                        self.pub_points(self.old_points)


                self.get_logger().info(f"{points}")
                self.get_logger().info(f"time (s): {end-start}")
                if self.dsef.star2_img is not None:
                    if self.dsef.center_image is not None:
                        cv2.imshow("DSEG", self.dsef.center_image)
                    cv2.imshow("All points", self.dsef.star2_img)
                    cv2.waitKey(1)  # Add a delay to allow the image to be rendered
                    
        
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')

        except Exception as e:
            self.get_logger().error(f'Something else happened: {e}')
            traceback.print_exc()



    # ------ Publisher function ------ #

    def pub_points(self, pos: List[Tuple[float, float]]) -> None:
        msg = Polygon()
        start = Point32()
        end = Point32()
        start.x, start.y = pos[1]
        msg.points.append(start)
        end.x, end.y = pos[0]
        msg.points.append(end)
        self.tilt_angle_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    dsef_node = DSEFNode()
    rclpy.spin(dsef_node)
    dsef_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()