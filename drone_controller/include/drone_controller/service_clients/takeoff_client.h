#ifndef TAKEOFF_CLIENT_H
#define TAKEOFF_CLIENT_H

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_tol.hpp"

class TakeoffClient : public rclcpp::Node
{
public:
    TakeoffClient();
    void send_takeoff_request();
    bool is_success() const;
    bool is_error() const;

private:
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr client_;
    bool success_;
    bool error_;
};

#endif // TAKEOFF_CLIENT_H