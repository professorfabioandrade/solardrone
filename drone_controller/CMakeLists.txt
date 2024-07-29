cmake_minimum_required(VERSION 3.5)
project(drone_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(airsim_interfaces REQUIRED)

include_directories(include)

add_library(takeoff_client src/service_clients/takeoff_client.cpp)
ament_target_dependencies(takeoff_client rclcpp mavros_msgs)

add_library(land_client src/service_clients/land_client.cpp)
ament_target_dependencies(land_client rclcpp mavros_msgs)

add_library(set_mode_client src/service_clients/set_mode_client.cpp)
ament_target_dependencies(set_mode_client rclcpp mavros_msgs)

add_library(arming_client src/service_clients/arming_client.cpp)
ament_target_dependencies(arming_client rclcpp mavros_msgs)

add_executable(drone_controller_node 
  src/drone_controller_node.cpp 
  src/drone_controller.cpp 
  src/service_clients/takeoff_client.cpp 
  src/service_clients/land_client.cpp
  src/service_clients/set_mode_client.cpp
  src/service_clients/arming_client.cpp) 
ament_target_dependencies(drone_controller_node rclcpp rclcpp_action std_msgs std_srvs sensor_msgs nav_msgs mavros_msgs airsim_interfaces)

install(TARGETS
  takeoff_client
  land_client
  drone_controller_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()