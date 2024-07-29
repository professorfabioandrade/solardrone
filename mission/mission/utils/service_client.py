import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ServiceClient:
    def __init__(self, node: Node, service_name: str) -> None:
        self.service_name = service_name
        self.node = node
        self.client_ = self.node.create_client(Trigger, self.service_name)

    def send_request(self) -> Trigger.Response:
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'{self.service_name} service not available, waiting again...')
        
        trigger = Trigger.Request()
        future = self.client_.call_async(trigger)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()
