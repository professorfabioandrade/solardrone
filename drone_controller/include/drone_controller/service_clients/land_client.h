#ifndef LAND_CLIENT_H
#define LAND_CLIENT_H

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_tol.hpp"

class LandClient : public rclcpp::Node
{
public:
    LandClient();
    void send_land_request();
    bool is_success() const;
    bool is_error() const;

private:
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr client_;
    bool success_;
    bool error_;
};

#endif // LAND_CLIENT_H