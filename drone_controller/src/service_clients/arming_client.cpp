#include "drone_controller/service_clients/arming_client.h"

ArmingClient::ArmingClient()
: Node("arming_client")
{
    client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

    success_ = false;
    error_ = false;
}

void ArmingClient::arm_vehicle()
{
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Service not available.");
        return;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;

    auto result_future = client_->async_send_request(request);

    try {
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive Arming service response");
            error_=true;
            return;
        }
        auto result = result_future.get();
        if (result->success) {
            RCLCPP_INFO(this->get_logger(), "Vehicle armed successfully.");
            success_ = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm vehicle.");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        error_=true;
    }
}

bool ArmingClient::is_success() const {
    return this->success_;
}

bool ArmingClient::is_error() const {
    return this->error_;
}