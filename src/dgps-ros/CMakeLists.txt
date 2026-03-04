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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

include_directories(
  include
  ${LIBSERIAL_INCLUDE_DIRS}
)

add_Library(${PROJECT_NAME}
  src/serial.cpp
  src/differential_gps.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${LIBSERIAL_LIBRARIES}
)

add_executable(dgps_test test/test_serial.cpp)
target_link_libraries(dgps_test
  ${PROJECT_NAME}
)

add_executable(dgps_node 
  ros/dgps_node.cpp
  ros/node.cpp
)
ament_target_dependencies(dgps_node
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  rtcm_msgs
)
target_link_libraries(dgps_node
  ${PROJECT_NAME}
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
  dgps_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
