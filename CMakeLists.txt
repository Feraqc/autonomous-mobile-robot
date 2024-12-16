cmake_minimum_required(VERSION 3.5)

# Define the project name and specify the language(s) used
project(mobile_robot LANGUAGES CXX)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
add_executable(cpp_node mobile_robot/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp std_msgs geometry_msgs nav_msgs tf2 sensor_msgs)

# Specify dependencies
ament_target_dependencies(cpp_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
)

install(TARGETS cpp_node
  DESTINATION lib/${PROJECT_NAME})


# Install Python scripts
install(PROGRAMS
  mobile_robot/cmd_publisher_node.py
  mobile_robot/robot_data.py
  mobile_robot/tcp_server_node.py

  DESTINATION lib/${PROJECT_NAME})

# Finalize
ament_package()
