#include "drone_controller/service_clients/set_mode_client.h"

SetModeClient::SetModeClient()
: Node("set_mode_client")
{
    client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    success_ = false;
    error_ = false;
}

void SetModeClient::set_mode()
{
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Service not available.");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Setting mode to GUIDED...");
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "GUIDED";

    auto result_future = client_->async_send_request(request);

    try {
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive Set mode service response");
            error_=true;
            return;
        }
        auto result = result_future.get();
        if (result->mode_sent) {
            RCLCPP_INFO(this->get_logger(), "Set mode request sent successfully.");
            success_ = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to send mode request.");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        error_=true;
    }
}

bool SetModeClient::is_success() const {
    return this->success_;
}

bool SetModeClient::is_error() const {
    return this->error_;
}