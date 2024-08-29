#include "drone_controller/drone_controller.h"

DroneController::DroneController()
: Node("drone_controller")
{
    takeoff_handler_server_ = this->create_service<std_srvs::srv::Trigger>(
        "/drone_controller/takeoff_trigger", 
        std::bind(&DroneController::takeoff_handler_callback, this, std::placeholders::_1, std::placeholders::_2));

    landing_handler_server_ = this->create_service<std_srvs::srv::Trigger>(
        "/drone_controller/landing_trigger", 
        std::bind(&DroneController::landing_handler_callback, this, std::placeholders::_1, std::placeholders::_2));

    pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/drone_controller/pos_to_send", 10, 
        std::bind(&DroneController::pos_request_callback, this, std::placeholders::_1));

    pos_vel_sub_ = this->create_subscription<mavros_msgs::msg::PositionTarget>(
        "/drone_controller/pos_vel_to_send", 10, 
        std::bind(&DroneController::pos_vel_request_callback, this, std::placeholders::_1));

    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
    local_pos_pub_raw_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);

    takeoff_client_ = std::make_shared<TakeoffClient>();
    land_client_ = std::make_shared<LandClient>();
    set_mode_client_ = std::make_shared<SetModeClient>();
    arming_client_ = std::make_shared<ArmingClient>();

    std::this_thread::sleep_for(std::chrono::seconds(1));
    set_guided_mode();
    arm_drone();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(this->get_logger(), "Ready to control the drone!");
}

void DroneController::pos_request_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    send_to_pos(*msg);
}

void DroneController::pos_vel_request_callback(const mavros_msgs::msg::PositionTarget::SharedPtr msg)
{
    send_to_pos_with_vel(*msg);
}

void DroneController::takeoff_handler_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Takeoff Requested!");

    if (takeoff_client_->is_success()) {
        RCLCPP_WARN(this->get_logger(), "Take off already performed!");
        response->success = false;
        response->message = "Take off already performed!";
        return;
    }

    takeoff_client_->send_takeoff_request();

    while (!takeoff_client_->is_success()) {
        if (takeoff_client_->is_error()) {
            RCLCPP_ERROR(this->get_logger(), "Error during take off!");
            response->success = false;
            response->message = "Error during take off!";
            return;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::this_thread::sleep_for(std::chrono::seconds(10));
    response->success = true;
    response->message = "Take off successful!";
}

void DroneController::landing_handler_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Landing Requested!");

    if (land_client_->is_success()) {
        RCLCPP_WARN(this->get_logger(), "Landing already performed!");
        response->success = false;
        response->message = "Landing already performed!";
        return;
    }

    land_client_->send_land_request();

    while (!land_client_->is_success()) {
        if (land_client_->is_error()) {
            RCLCPP_ERROR(this->get_logger(), "Error during landing!");
            response->success = false;
            response->message = "Error during landing!";
            return;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::this_thread::sleep_for(std::chrono::seconds(10));
    response->success = true;
    response->message = "Landing successful!";
}

void DroneController::send_to_pos(const geometry_msgs::msg::PoseStamped &pose)
{
    local_pos_pub_->publish(pose);
    RCLCPP_INFO(this->get_logger(), "Position sent.");
}
void DroneController::send_to_pos_with_vel(const mavros_msgs::msg::PositionTarget &pose)
{
    local_pos_pub_raw_->publish(pose);
    RCLCPP_INFO(this->get_logger(), "Position sent.");
}

void DroneController::set_guided_mode()
{
    RCLCPP_INFO(this->get_logger(), "Setting drone mode to GUIDED!");
    
    do {
        set_mode_client_->set_mode();
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (set_mode_client_->is_error())
            return;
    } while (!set_mode_client_->is_success());
}

void DroneController::arm_drone()
{
    RCLCPP_INFO(this->get_logger(), "Arming drone!");
    
    do {
        arming_client_->arm_vehicle();
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (arming_client_->is_error())
            return;
    } while (!arming_client_->is_success());
}