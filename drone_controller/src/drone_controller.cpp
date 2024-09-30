#include "drone_controller/drone_controller.h"

DroneController::DroneController()
: Node("drone_controller")
{
    process_parameters();
    initialize_publishers();
    initialize_subscribers();
    initialize_services();

    std::this_thread::sleep_for(std::chrono::seconds(1));
    set_guided_mode();
    arm_drone();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(this->get_logger(), "Ready to control the drone!");
}

/* ------ Initializing Components ------ */
    
void DroneController::initialize_publishers(){
    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
    local_pos_pub_raw_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);
    gimbal_pub_ = this->create_publisher<airsim_interfaces::msg::GimbalAngleEulerCmd>("/airsim_node/gimbal_angle_euler_cmd", 10);
}
   
void DroneController::initialize_subscribers() {
    pos_vel_sub_ = this->create_subscription<mavros_msgs::msg::PositionTarget>(
        waypoint_topic_name.c_str(), 10, 
        std::bind(&DroneController::pos_vel_request_callback, this, std::placeholders::_1));

    gimbal_sub_ = this->create_subscription<airsim_interfaces::msg::GimbalAngleEulerCmd>(
        gimbal_topic_name.c_str(), 10, 
        std::bind(&DroneController::gimbal_callback, this, std::placeholders::_1));
}
    
void DroneController::initialize_services(){
    takeoff_handler_server_ = this->create_service<std_srvs::srv::Trigger>(
        takeoff_service_name.c_str(), 
        std::bind(&DroneController::takeoff_handler_callback, this, std::placeholders::_1, std::placeholders::_2));

    landing_handler_server_ = this->create_service<std_srvs::srv::Trigger>(
        landing_service_name.c_str(), 
        std::bind(&DroneController::landing_handler_callback, this, std::placeholders::_1, std::placeholders::_2));


    takeoff_client_ = std::make_shared<TakeoffClient>(takeoff_altitude);
    land_client_ = std::make_shared<LandClient>();
    set_mode_client_ = std::make_shared<SetModeClient>();
    arming_client_ = std::make_shared<ArmingClient>();
}

/* ------ Processing Parameters ------ */

void DroneController::process_parameters(){
    this->declare_parameter<double>("takeoff_altitude", 1.0);
    this->declare_parameter<std::string>("takeoff_service_name", "/service");
    this->declare_parameter<std::string>("landing_service_name", "/service");
    this->declare_parameter<std::string>("waypoint_topic_name", "/topic");
    this->declare_parameter<std::string>("gimbal_topic_name", "/topic");

    takeoff_altitude = this->get_parameter("takeoff_altitude").as_double();
    takeoff_service_name = this->get_parameter("takeoff_service_name").as_string();
    landing_service_name = this->get_parameter("landing_service_name").as_string();
    waypoint_topic_name = this->get_parameter("waypoint_topic_name").as_string();
    gimbal_topic_name = this->get_parameter("gimbal_topic_name").as_string();
}

/* ------ Handling Callbacks ------ */

void DroneController::pos_request_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    send_to_pos(*msg);
}

void DroneController::pos_vel_request_callback(const mavros_msgs::msg::PositionTarget::SharedPtr msg)
{
    send_to_pos_with_vel(*msg);
}

void DroneController::gimbal_callback(const airsim_interfaces::msg::GimbalAngleEulerCmd::SharedPtr msg){
    send_to_gimbal(*msg);
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

/* ------ Publishers function ------ */

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

void DroneController::send_to_gimbal(const airsim_interfaces::msg::GimbalAngleEulerCmd &gimbal)
{
    gimbal_pub_->publish(gimbal);
    RCLCPP_INFO(this->get_logger(), "Gimbal Position sent.");
}

/* ------ Services function ------ */

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