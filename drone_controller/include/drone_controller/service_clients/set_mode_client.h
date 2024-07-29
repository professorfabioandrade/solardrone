#ifndef SET_MODE_CLIENT_H
#define SET_MODE_CLIENT_H

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

class SetModeClient : public rclcpp::Node
{
public:
    SetModeClient();
    void set_mode();
    bool is_success() const;
    bool is_error() const;

private:
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client_;
    bool success_;
    bool error_;
};

#endif // SET_MODE_CLIENT_H