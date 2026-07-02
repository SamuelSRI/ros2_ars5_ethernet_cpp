cmake_minimum_required(VERSION 3.10)
project(ros2_ars5_ethernet_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

include_directories(include)

add_executable(ars5_node
  src/ars5_node.cpp
  src/udp_receiver.cpp
  src/ars5_decoder.cpp
)

ament_target_dependencies(ars5_node
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(ars5_fusion_node
  src/ars5_fusion_node.cpp
)

ament_target_dependencies(ars5_fusion_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
)

install(TARGETS
  ars5_node
  ars5_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
