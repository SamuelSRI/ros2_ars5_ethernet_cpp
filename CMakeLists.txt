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

add_executable(ars5_node
  src/ars5_node.cpp
  src/udp_receiver.cpp
  src/ars5_decoder.cpp
)

target_include_directories(ars5_node PUBLIC
  include
)

target_link_libraries(ars5_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

add_executable(ars5_fusion_node
  src/ars5_fusion_node.cpp
)

target_include_directories(ars5_fusion_node PUBLIC
  include
)

target_link_libraries(ars5_fusion_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  ${tf2_msgs_TARGETS}
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
