import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from solar_panels_img_processing.utils.georefering import Georeferencing



class GeoreferenceNode(Node):

    def __init__(self):
        super().__init__('georef_node')
        self.georeferencing = Georeferencing()

        self.line_georef_pub = self.create_publisher(Polygon, '/georef_line', 10)
        self.create_subscription(Polygon, '/detected_line', self.line_callback, 10)
        
        qos_profile = self.create_qos_profile()
        
        self.create_subscription(Odometry, '/mavros/global_position/local', self.global_position_callback, qos_profile)
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.compass_hdg_callback, qos_profile)

    def create_qos_profile(self) -> QoSProfile:
        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

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

        P_ENU_start = self.georeferencing(u1, v1)
        P_ENU_end = self.georeferencing(u2, v2)
        self.publish_georeferenced_line(P_ENU_start, P_ENU_end)

    def publish_georeferenced_line(self, start: np.ndarray, end: np.ndarray) -> None:
        polygon = Polygon()
        polygon.points.extend([self.create_point32(start), self.create_point32(end)])
        self.line_georef_pub.publish(polygon)

    @staticmethod
    def create_point32(coord: np.ndarray) -> Point32:
        print(coord)
        return Point32(x=float(coord[0]), y=float(coord[1]), z=1.8)


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
