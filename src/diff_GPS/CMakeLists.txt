cmake_minimum_required(VERSION 3.8)
project(diff_GPS)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(diff_GPS_node src/diff_GPS_node.cpp)
ament_target_dependencies(diff_GPS_node rclcpp sensor_msgs geometry_msgs serial)

# Install Cpp executables
install(TARGETS
  diff_GPS_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()