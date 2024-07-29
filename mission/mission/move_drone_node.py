import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mission.utils.coordinate_converter import CoordinateConverter
from mission.utils.multiprocessing_handler import MultiprocessingHandler
from mission.utils.service_client import ServiceClient
from time import sleep
from multiprocessing import Manager

class MoveDroneNode(Node):
    def __init__(self, manager) -> None:
        super().__init__('drone_controller_client')

        self.process = MultiprocessingHandler(manager)
        self.local_x, self.local_y, self.local_z = self.process.parallel_float_values(3)
        self.process.lock()

        self.takeoff_client = ServiceClient(self, '/drone_controller/takeoff_trigger')
        self.landing_client = ServiceClient(self, '/drone_controller/landing_trigger')

        self.goal_pub = self.create_publisher(PoseStamped, '/drone_controller/pos_to_send', 10)

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.local_sub = self.create_subscription(
            Odometry,
            '/mavros/global_position/local',
            self.local_callback,
            qos_profile
        )

        self.coord_conv_ = CoordinateConverter(-5.038051, -37.789214, 416)
        self.run()

    def local_callback(self, msg: Odometry) -> None:
        self.process.set_value(self.local_x, msg.pose.pose.position.x)
        self.process.set_value(self.local_y, msg.pose.pose.position.y)
        self.process.set_value(self.local_z, msg.pose.pose.position.z)

    def run(self) -> None:
        self.get_logger().info('Performing Takeoff!')
        response_to = self.takeoff_client.send_request()
        if not response_to.success:
            if response_to.message == "Take off already performed!":
                self.get_logger().warn("Take off already performed!")
                sleep(3)
            else:
                raise Exception(f"Error: {response_to.message}")

        self.process.run(
            target=self.goals_, 
            args=(self.local_x, self.local_y, self.local_z, self.process.local_data_lock)
        )

    def landing(self) -> None:
        self.get_logger().info('Performing Landing!')
        response_land = self.landing_client.send_request()
        if not response_land.success:
            raise Exception(f"Error: {response_land.message}")

    def goals_(self, local_x, local_y, local_z, local_data_lock) -> None:
        self.get_logger().info('Sending goals!')
        self.send_goal(-5.03823044, -37.78912143, 421, local_x, local_y, local_z, local_data_lock)
        self.send_goal(-5.03955195, -37.78912143, 421, local_x, local_y, local_z, local_data_lock)
        self.send_goal(-5.03955195, -37.78918744, 421, local_x, local_y, local_z, local_data_lock)
        self.send_goal(-5.03825317, -37.78918744, 421, local_x, local_y, local_z, local_data_lock)
        self.send_goal(-5.03809181, -37.78921905, 421, local_x, local_y, local_z, local_data_lock)
        self.landing()

    def send_goal(self, lat: float, lon: float, alt: float, local_x, local_y, local_z, local_data_lock) -> None:
        goal_msg = self.coords2PoseStamped(lat, lon, alt)
        self.goal_pub.publish(goal_msg)
        self.get_logger().info('Goal sent!')
        while True:
            with local_data_lock:
                if (abs(local_x.value - goal_msg.pose.position.x) < 1.0 and
                    abs(local_y.value - goal_msg.pose.position.y) < 1.0 and
                    abs(local_z.value - goal_msg.pose.position.z) < 1.0):
                    break
            self.get_logger().info(" ### ")
            self.get_logger().info(f'Goal: ({goal_msg.pose.position.x}, {goal_msg.pose.position.y}, {goal_msg.pose.position.z})')
            self.get_logger().info(f'Curr: ({local_x.value}, {local_y.value}, {local_z.value})')
            sleep(1)

    def coords2PoseStamped(self, lat: float, lon: float, alt: float) -> PoseStamped:
        x_, y_, z_ = self.coord_conv_.convert(lat, lon, alt)

        self.get_logger().info(f'Goal coords: lat:{lat} | lon: {lon}')
        self.get_logger().info(f'(x,y,z) value: ({x_}, {y_}, {z_})')

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.position.x = x_
        goal_msg.pose.position.y = y_
        goal_msg.pose.position.z = z_
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        return goal_msg

def main(args=None):
    rclpy.init(args=args)
    manager = Manager()
    drone_controller_client = MoveDroneNode(manager)
    rclpy.spin(drone_controller_client)

if __name__ == '__main__':
    main()
