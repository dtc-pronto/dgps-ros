cmake_minimum_required(VERSION 3.8)
project(dgps)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(dgps_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(glog REQUIRED)
find_package(tf2 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

set(node_plugins "")

include_directories(
  include
  ${LIBSERIAL_INCLUDE_DIRS}
)

add_Library(${PROJECT_NAME} SHARED
  src/serial.cpp
  src/differential_gps.cpp
)

target_link_libraries(${PROJECT_NAME}
  glog::glog
  ${LIBSERIAL_LIBRARIES}
)

add_executable(dgps_test test/test_serial.cpp)
target_link_libraries(dgps_test
  ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}_ros SHARED
  ros/dgps_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_ros
  rclcpp 
  rclcpp_components
  sensor_msgs 
  geometry_msgs 
  rtcm_msgs
  dgps_msgs
  std_msgs
  tf2
)
target_link_libraries(${PROJECT_NAME}_ros
  ${PROJECT_NAME}
)

rclcpp_components_register_nodes(${PROJECT_NAME}_ros "dgps::DGPSNode")
set(node_plugins "${node_plugins}dgps::DGPSNode;$<TARGET_FILE:${PROJECT_NAME}_ros>\n")

add_executable(${PROJECT_NAME}_node
  ros/node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  rclcpp::rclcpp
  ${PROJECT_NAME}
  ${PROJECT_NAME}_ros
)


# Create Cpp executable
add_executable(diff_gps_node ros/diff_gps_node.cpp)
ament_target_dependencies(diff_gps_node
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  rtcm_msgs
)
target_link_libraries(diff_gps_node 
  ${LIBSERIAL_LIBRARIES}
)

# Install Cpp executables
install(TARGETS
  diff_gps_node
  ${PROJECT_NAME}
  ${PROJECT_NAME}_ros
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
