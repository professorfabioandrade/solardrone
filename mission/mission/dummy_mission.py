import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Empty, Float64
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget
from airsim_interfaces.msg import GimbalAngleEulerCmd
from mission.utils.coordinate_converter import CoordinateConverter
from mission.utils.threading_handler import ThreadingHandler
from mission.utils.service_client import ServiceClient
from mission.utils.csv_handler import CSVHandler
from time import sleep, time_ns
from math import pi
from json import loads

class MoveDroneNode(Node):
    def __init__(self) -> None:
        super().__init__('move_drone_node')

        self.process_parameters()
        self.initialize_components()
        self.initialize_publishers()
        self.initialize_subscribers()
        self.initialize_services()
        
        self.coord_converter = CoordinateConverter(self.initial_latitude, self.initial_longitude, self.initial_altitude)
        self.log_csv = CSVHandler(folder_path="/home/ubuntu/", filename="dummy_data.csv", header=["Time","Pos_x", "Pos_y", "Psi_ENU_rad"])
                
        self.run_mission()

    # ------ Initializing the components ------ #

    def initialize_components(self) -> None:
        self.process = ThreadingHandler()
        self.local_x, self.local_y, self.local_z = self.process.parallel_float_values(3)
        self.process.lock()

    def initialize_publishers(self) -> None:
        self.goal_pub = self.create_publisher(PositionTarget, self.waypoint_topic_name, 10)
        self.cam_trigger_pub = self.create_publisher(Empty, self.camera_trigger_topic_name, 10)
        self.gimbal_pub = self.create_publisher(GimbalAngleEulerCmd, self.gimbal_topic_name, 10)

    def initialize_subscribers(self) -> None:
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float64, self.compass_hdg_topic_name, self.compass_hdg_callback, qos_profile)
        self.local_sub = self.create_subscription(Odometry, self.local_position_topic_name, self.local_callback, qos_profile)

    def initialize_services(self) -> None:
        self.takeoff_client = ServiceClient(self, self.takeoff_service_name)
        self.landing_client = ServiceClient(self, self.landing_service_name)

    # ------ Processing Parameters ------ #

    def process_parameters(self) -> None:
        param_names = [
            ("latitude", rclpy.Parameter.Type.DOUBLE), ("longitude", rclpy.Parameter.Type.DOUBLE), 
            ("altitude", rclpy.Parameter.Type.DOUBLE), ("flight_altitude", rclpy.Parameter.Type.DOUBLE), 
            ("yaw", rclpy.Parameter.Type.DOUBLE), ("waypoint_start_buffer", rclpy.Parameter.Type.DOUBLE), 
            ("waypoint_end_buffer_x", rclpy.Parameter.Type.DOUBLE), ("waypoint_end_buffer_y", rclpy.Parameter.Type.DOUBLE),
            ("panel_tilt", rclpy.Parameter.Type.DOUBLE), ("use_waypoints_utm", rclpy.Parameter.Type.BOOL), 
            ("landing_service_name", rclpy.Parameter.Type.STRING), ("waypoint_topic_name", rclpy.Parameter.Type.STRING),
            ("camera_trigger_topic_name", rclpy.Parameter.Type.STRING), ("local_position_topic_name", rclpy.Parameter.Type.STRING),
            ("gimbal_topic_name", rclpy.Parameter.Type.STRING), ("takeoff_service_name", rclpy.Parameter.Type.STRING), 
            ("compass_hdg_topic_name", rclpy.Parameter.Type.STRING), ("waypoints", rclpy.Parameter.Type.STRING),
        ]
        
        for param, type_ in param_names:
            self.declare_parameter(param, type_)

        self.initial_latitude = self.get_parameter("latitude").get_parameter_value().double_value
        self.initial_longitude = self.get_parameter("longitude").get_parameter_value().double_value
        self.initial_altitude = self.get_parameter("altitude").get_parameter_value().double_value
        self.flight_altitude = self.get_parameter("flight_altitude").get_parameter_value().double_value
        self.yaw = self.get_parameter("yaw").get_parameter_value().double_value
        self.panel_tilt = self.get_parameter("panel_tilt").get_parameter_value().double_value

        self.waypoint_start_buffer = self.get_parameter("waypoint_start_buffer").get_parameter_value().double_value
        self.waypoint_end_buffer_x = self.get_parameter("waypoint_end_buffer_x").get_parameter_value().double_value
        self.waypoint_end_buffer_y = self.get_parameter("waypoint_end_buffer_y").get_parameter_value().double_value
        self.use_waypoints_utm = self.get_parameter("use_waypoints_utm").get_parameter_value().bool_value

        self.takeoff_service_name = self.get_parameter("takeoff_service_name").get_parameter_value().string_value
        self.landing_service_name = self.get_parameter("landing_service_name").get_parameter_value().string_value
        self.waypoint_topic_name = self.get_parameter("waypoint_topic_name").get_parameter_value().string_value
        self.camera_trigger_topic_name = self.get_parameter("camera_trigger_topic_name").get_parameter_value().string_value
        self.local_position_topic_name = self.get_parameter("local_position_topic_name").get_parameter_value().string_value
        self.compass_hdg_topic_name = self.get_parameter("compass_hdg_topic_name").get_parameter_value().string_value
        self.gimbal_topic_name = self.get_parameter("gimbal_topic_name").get_parameter_value().string_value

        self.waypoints = loads(self.get_parameter("waypoints").get_parameter_value().string_value)

    # ------ Handling Callbacks ------ #

    def local_callback(self, msg: Odometry) -> None:
        self.process.set_value(self.local_x, msg.pose.pose.position.x)
        self.process.set_value(self.local_y, msg.pose.pose.position.y)
        self.process.set_value(self.local_z, msg.pose.pose.position.z)

    def compass_hdg_callback(self, msg: Float64) -> None:
        self.compass = msg.data

    # ------ Handling Mission ------ #

    def run_mission(self) -> None:
        self.get_logger().info('Starting the mission: Takeoff initiated.')
        self.perform_takeoff()
        self.process.run(target=self.process_waypoints, args=(self.local_x, self.local_y, self.local_z, self.process.local_data_lock))
       

    def perform_takeoff(self) -> None:
        response = self.takeoff_client.send_request()
        if not response.success:
            if response.message == "Take off already performed!":
                self.get_logger().warn("Take off already performed!")
                sleep(3)
            else:
                raise Exception(f"Takeoff failed: {response.message}")

    def process_waypoints(self, local_x, local_y, local_z, local_data_lock):
        self.get_logger().info('Processing waypoints.')
        if not self.waypoints:
            self.get_logger().warn("No waypoints to send.")
            return
        
        for index, waypoint in enumerate(self.waypoints):
            is_start_point = index % 2 == 0 and len(self.waypoints) > index + 1
            self.send_goal(waypoint, local_x, local_y, local_z, local_data_lock, cam_trigger=not is_start_point, is_start_point=is_start_point)

        self.perform_landing(local_data_lock)

    def perform_landing(self, local_data_lock):
        self.get_logger().info('Landing the drone.')
        with local_data_lock:
            response = self.landing_client.send_request()
            if not response.success:
                raise Exception(f"Landing failed: {response.message}")

    def send_goal(self, waypoint, local_x, local_y, local_z, local_data_lock, cam_trigger=False, is_start_point=False):
        goal_msg = self.convert_waypoint_to_goal(waypoint)
        self.goal_pub.publish(goal_msg)
        self.get_logger().info('Goal sent.')

        buffer_x, buffer_y = (self.waypoint_start_buffer, self.waypoint_start_buffer) if is_start_point else (self.waypoint_end_buffer_x, self.waypoint_end_buffer_y)
        self.wait_until_goal_reached(goal_msg, local_x, local_y, local_z, local_data_lock, buffer_x, buffer_y, cam_trigger)

    
    # ------ Publishers Functions ------ #

    def publish_gimbal(self) -> None:
        g_msg = self.gimbal_to_msg()
        self.gimbal_pub.publish(g_msg)

    # ------ Operations ------ #

    def wait_until_goal_reached(self, goal_msg: PositionTarget, local_x, local_y, local_z, local_data_lock, buffer_x: float, buffer_y: float, cam_trigger: bool) -> None:
        while True:
            with local_data_lock:
                if self.is_goal_reached(goal_msg, local_x, local_y, local_z, buffer_x, buffer_y):
                    break
            if cam_trigger:
                self.publish_gimbal()
                self.cam_trigger_pub.publish(Empty())
            self.log_csv.add_line([time_ns(), local_x[0], local_y[0], self.compass_to_psi_enu(self.compass)])
            sleep(0.1)

    def is_goal_reached(self, goal_msg, local_x, local_y, local_z, buffer_x, buffer_y):
        
        self.get_logger().info(f'(x,y,z) value: ({local_x[0]}, {local_y[0]}, {local_z[0]})')
        return (
            abs(local_x[0] - goal_msg.position.x) < buffer_x and
            abs(local_y[0] - goal_msg.position.y) < buffer_y and
            abs(local_z[0] - goal_msg.position.z) < 1.0
        )
    
    @staticmethod
    def compass_to_psi_enu(compass: float) -> None:
        # Save the heading (psi) from the message [degrees] in NED frame
        psi_NED = compass * pi / 180
        return (pi / 2 - psi_NED) % (2 * pi)
    
    # ------ Generate Messages ------ #

    def gimbal_to_msg(self) -> GimbalAngleEulerCmd:
        gimbal_msg = GimbalAngleEulerCmd()
                
        gimbal_msg.camera_name = 'camera_1'
        gimbal_msg.vehicle_name = 'Drone_1'
        gimbal_msg.pitch = -self.panel_tilt # funciona apenas para manhã, mudar lógica para caso geral
        gimbal_msg.yaw = self.compass # graus mesmo
        return gimbal_msg
     
    def convert_waypoint_to_goal(self, waypoint: dict) -> PositionTarget:
        if self.use_waypoints_utm:
            return self.local2PositionTarget(waypoint['x'] - self.coord_converter.x0, waypoint['y'] - self.coord_converter.y0, self.flight_altitude, self.yaw * pi / 180)
        else:
            return self.coords2PositionTarget(waypoint['lat'], waypoint['lon'], self.flight_altitude)

    def coords2PositionTarget(self, lat: float, lon: float, alt: float) -> PositionTarget:
        x, y, z = self.coord_converter.convert_to_relative_utm(lat, lon, alt)
        return self.create_position_target_message(x, y, z)

    def local2PositionTarget(self, x: float, y: float, z: float, psi: float) -> PositionTarget:
        return self.create_position_target_message(x, y, z-self.initial_altitude, psi)

    def create_position_target_message(self, x, y, z, yaw=None) -> PositionTarget:
        goal_msg = PositionTarget()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.coordinate_frame = 1  # LOCAL_NED
        goal_msg.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | 
                         PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX | 
                         PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | 
                         PositionTarget.IGNORE_YAW_RATE)
        goal_msg.position.x = float(x)
        goal_msg.position.y = float(y)
        goal_msg.position.z = float(z)
        if yaw is not None:
            goal_msg.yaw = float(yaw)
        return goal_msg

def main(args=None):
    rclpy.init(args=args)
    node = MoveDroneNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
