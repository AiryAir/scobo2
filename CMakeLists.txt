cmake_minimum_required(VERSION 3.5)
project(scobo2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build ROS 2 node executable
add_executable(airduino_combined_ros2_cpp airduino_combined_ros2.cpp)
ament_target_dependencies(airduino_combined_ros2_cpp rclcpp std_msgs)

# Install
install(
  TARGETS
  airduino_combined_ros2_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  config
  description
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

