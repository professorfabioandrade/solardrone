#ifndef ARMING_CLIENT_H
#define ARMING_CLIENT_H

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_bool.hpp"

class ArmingClient : public rclcpp::Node
{
public:
    ArmingClient();
    void arm_vehicle();
    bool is_success() const;
    bool is_error() const;

private:
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr client_;
    bool success_;
    bool error_;
};

#endif // ARMING_CLIENT_H