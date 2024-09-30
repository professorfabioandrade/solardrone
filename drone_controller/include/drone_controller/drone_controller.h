#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "airsim_interfaces/msg/gimbal_angle_euler_cmd.hpp"
#include "drone_controller/service_clients/takeoff_client.h"
#include "drone_controller/service_clients/land_client.h"
#include "drone_controller/service_clients/set_mode_client.h"
#include "drone_controller/service_clients/arming_client.h"

class DroneController : public rclcpp::Node
{
public:
    DroneController();

private:

    /* ------ Initializing Components ------ */
    void initialize_publishers();
    void initialize_subscribers();
    void initialize_services();

    /* ------ Processing Parameters ------ */
    void process_parameters();

    /* ------ Handling Callbacks ------ */
    void pos_request_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void pos_vel_request_callback(const mavros_msgs::msg::PositionTarget::SharedPtr msg);
    void gimbal_callback(const airsim_interfaces::msg::GimbalAngleEulerCmd::SharedPtr msg);    
    void takeoff_handler_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void landing_handler_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    /* ------ Publishers function ------ */
    void send_to_pos(const geometry_msgs::msg::PoseStamped &pose);
    void send_to_pos_with_vel(const mavros_msgs::msg::PositionTarget &pose);
    void send_to_gimbal(const airsim_interfaces::msg::GimbalAngleEulerCmd &gimbal);

    /* ------ Services function ------ */
    void set_guided_mode();
    void arm_drone();

    /* ------ Publishers ------ */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr local_pos_pub_raw_;
    rclcpp::Publisher<airsim_interfaces::msg::GimbalAngleEulerCmd>::SharedPtr gimbal_pub_;
    
    /* ------ Subscribers ------ */
    rclcpp::Subscription<airsim_interfaces::msg::GimbalAngleEulerCmd>::SharedPtr gimbal_sub_;
    rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr pos_vel_sub_;
    
    /* ------ Services ------ */
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_handler_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr landing_handler_server_;

    /* ------ Service Clients ------ */
    std::shared_ptr<TakeoffClient> takeoff_client_;
    std::shared_ptr<LandClient> land_client_;
    std::shared_ptr<SetModeClient> set_mode_client_;
    std::shared_ptr<ArmingClient> arming_client_;

    double takeoff_altitude;
    std::string takeoff_service_name;
    std::string landing_service_name;
    std::string waypoint_topic_name;
    std::string gimbal_topic_name;
};

#endif // DRONE_CONTROLLER_H