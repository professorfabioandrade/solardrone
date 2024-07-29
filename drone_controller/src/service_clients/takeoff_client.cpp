#include "drone_controller/service_clients/takeoff_client.h"

TakeoffClient::TakeoffClient()
: Node("takeoff_client")
{
    client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

    success_ = false;
    error_ = false;
}

void TakeoffClient::send_takeoff_request()
{
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Service not available, waiting again...");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Starting Take-off");
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

    request->altitude = 5.0;
    
    auto result_future = client_->async_send_request(request);
    
    try {
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive Takeoff service response");
            error_=true;
            return;
        }
        auto result = result_future.get();
        if (result->success) {
            RCLCPP_INFO(this->get_logger(), "Takeoff service call successful.");
            success_ = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Takeoff service call failed.");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        error_=true;
    }
}

bool TakeoffClient::is_success() const {
    return this->success_;
}

bool TakeoffClient::is_error() const {
    return this->error_;
}