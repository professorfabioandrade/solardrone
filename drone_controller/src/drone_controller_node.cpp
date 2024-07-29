#include "drone_controller/drone_controller.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}