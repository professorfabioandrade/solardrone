import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from airsim_interfaces.msg import GimbalAngleEulerCmd
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from solar_panels_img_processing.utils.georefering import Georeferencing
from json import loads


class GeoreferenceNode(Node):

    def __init__(self):
        super().__init__('georef_node')

        
        self.process_parameters()
        self.initialize_publishers()
        self.initialize_subscribers()

        self.georeferencing = Georeferencing(self.k_matrix, self.solar_panel_height, self.panel_tilt)

    # ------ Initializing the components ------ #
    
    def initialize_publishers(self) -> None:
        self.line_georef_pub = self.create_publisher(Polygon, self.georef_line_topic_name, 10)

    def initialize_subscribers(self) -> None:
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Polygon, self.canny_line_topic_name, self.line_callback, 10)

        self.create_subscription(Odometry, self.local_position_topic_name, self.global_position_callback, qos_profile)
        self.create_subscription(Float64, self.compass_hdg_topic_name, self.compass_hdg_callback, qos_profile)
        self.create_subscription(GimbalAngleEulerCmd, self.gimbal_topic_name, self.gimbal_angles_callback, 10)

    # ------ Processing Parameters ------ #

    def process_parameters(self) -> None:
        self.declare_parameter("solar_panel_height", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("panel_tilt", rclpy.Parameter.Type.DOUBLE)   

        self.declare_parameter("local_position_topic_name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("compass_hdg_topic_name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("georef_line_topic_name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("canny_line_topic_name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("gimbal_topic_name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("K_matrix", rclpy.Parameter.Type.STRING)

        self.solar_panel_height = self.get_parameter("solar_panel_height").get_parameter_value().double_value
        self.panel_tilt = self.get_parameter("panel_tilt").get_parameter_value().double_value     

        self.local_position_topic_name = self.get_parameter("local_position_topic_name").get_parameter_value().string_value
        self.compass_hdg_topic_name = self.get_parameter("compass_hdg_topic_name").get_parameter_value().string_value
        self.georef_line_topic_name = self.get_parameter("georef_line_topic_name").get_parameter_value().string_value
        self.canny_line_topic_name = self.get_parameter("canny_line_topic_name").get_parameter_value().string_value
        self.gimbal_topic_name = self.get_parameter("gimbal_topic_name").get_parameter_value().string_value

        self.k_matrix = loads(self.get_parameter("K_matrix").get_parameter_value().string_value)
            
    # ------ Handling Callbacks ------ #

    def global_position_callback(self, msg: Odometry) -> None:
        self.georeferencing.update_position([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

    def compass_hdg_callback(self, msg: Float64) -> None:
        self.georeferencing.update_heading(msg.data * math.pi / 180)

    def line_callback(self, msg: Polygon) -> None:
        start_point, end_point = msg.points[0], msg.points[1]
        u1, v1 = start_point.x, start_point.y
        u2, v2 = end_point.x, end_point.y

        try:
            P_ENU_start = self.georeferencing(u1, v1)
            P_ENU_end = self.georeferencing(u2, v2)
            self.publish_georeferenced_line(P_ENU_start, P_ENU_end)
        except Exception as e:
            self.get_logger().warn(f'{e}') 
    
    def gimbal_angles_callback(self, msg: GimbalAngleEulerCmd) -> None:
        self.georeferencing.update_gimbal(msg.roll, msg.pitch, msg.yaw)

    # ------ Publishers function ------ #

    def publish_georeferenced_line(self, start: np.ndarray, end: np.ndarray) -> None:
        polygon = Polygon()
        polygon.points.extend([self.create_point32(start), self.create_point32(end)])
        self.line_georef_pub.publish(polygon)

    # ------ Generate Messages ------ #

    def create_point32(self, coord: np.ndarray) -> Point32:
        print(coord)
        return Point32(x=float(coord[0]), y=float(coord[1]), z=self.solar_panel_height)


def main(args=None) -> None:
    rclpy.init(args=args)
    georef_node = GeoreferenceNode()

    try:
        rclpy.spin(georef_node)
    except KeyboardInterrupt:
        pass
    finally:
        georef_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
