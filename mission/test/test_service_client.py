import unittest
from unittest.mock import MagicMock, patch
from rclpy.node import Node
from mission.utils.service_client import ServiceClient

class TestServiceClient(unittest.TestCase):

    def setUp(self):
        self.node = MagicMock(spec=Node)
        self.client = ServiceClient(self.node, 'test_service')

    @patch('rclpy.client.Client.call_async')
    def test_send_request(self, mock_call_async):
        mock_future = MagicMock()
        mock_future.result.return_value.success = True
        mock_call_async.return_value = mock_future

        response = self.client.send_request()
        self.assertTrue(response.success)

if __name__ == '__main__':
    unittest.main()
