#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "drone_controller/service_clients/takeoff_client.h"
#include "drone_controller/service_clients/land_client.h"
#include "drone_controller/service_clients/set_mode_client.h"
#include "drone_controller/service_clients/arming_client.h"

class DroneController : public rclcpp::Node
{
public:
    DroneController();

private:
    void pos_request_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void send_to_pos(const geometry_msgs::msg::PoseStamped &pose);

    void takeoff_handler_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void landing_handler_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void set_guided_mode();
    void arm_drone();

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pos_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_handler_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr landing_handler_server_;

    std::shared_ptr<TakeoffClient> takeoff_client_;
    std::shared_ptr<LandClient> land_client_;
    std::shared_ptr<SetModeClient> set_mode_client_;
    std::shared_ptr<ArmingClient> arming_client_;
};

#endif // DRONE_CONTROLLER_H