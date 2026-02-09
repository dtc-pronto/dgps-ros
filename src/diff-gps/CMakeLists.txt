cmake_minimum_required(VERSION 3.8)
project(diff_gps)

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
find_package(rtcm_msgs REQUIRED)

find_library(SERIAL_LIB serial)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(diff_gps_node src/diff_gps_node.cpp)
ament_target_dependencies(diff_gps_node rclcpp sensor_msgs geometry_msgs rtcm_msgs)
target_link_libraries(diff_gps_node ${SERIAL_LIB})

# Install Cpp executables
install(TARGETS
  diff_gps_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()