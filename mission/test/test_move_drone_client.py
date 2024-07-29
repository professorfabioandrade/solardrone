import unittest
from unittest.mock import MagicMock, patch
from rclpy.node import Node
from mission.move_drone_client import DroneControllerNode

class TestDroneControllerNode(unittest.TestCase):

    @patch('mission.move_drone_client.ServiceClient')
    @patch('mission.move_drone_client.CoordinateConverter')
    def setUp(self, MockCoordinateConverter, MockServiceClient):
        self.mock_manager = MagicMock()
        self.node = DroneControllerNode(self.mock_manager)
        self.node.takeoff_client.send_request.return_value.success = True
        self.node.landing_client.send_request.return_value.success = True

    def test_takeoff(self):
        self.node.run()
        self.node.takeoff_client.send_request.assert_called_once()

    def test_landing(self):
        self.node.landing()
        self.node.landing_client.send_request.assert_called_once()

if __name__ == '__main__':
    unittest.main()
