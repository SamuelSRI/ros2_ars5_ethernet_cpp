cmake_minimum_required(VERSION 3.10)
project(ros2_ars5_ethernet_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(ROS_ROOT_DIR /opt/ros/lyrical)
set(ROS_INCLUDE_DIR ${ROS_ROOT_DIR}/include)
set(ROS_LIB_DIR ${ROS_ROOT_DIR}/lib)

add_executable(ars5_node
  src/ars5_node.cpp
  src/udp_receiver.cpp
  src/ars5_decoder.cpp
)

target_include_directories(ars5_node PUBLIC
  include
  ${ROS_INCLUDE_DIR}
  ${ROS_INCLUDE_DIR}/rclcpp
  ${ROS_INCLUDE_DIR}/sensor_msgs
  ${ROS_INCLUDE_DIR}/std_msgs
  ${ROS_INCLUDE_DIR}/builtin_interfaces
  ${ROS_INCLUDE_DIR}/rosidl_runtime_cpp
  ${ROS_INCLUDE_DIR}/rosidl_typesupport_cpp
  ${ROS_INCLUDE_DIR}/rcutils
  ${ROS_INCLUDE_DIR}/rcl
  ${ROS_INCLUDE_DIR}/rmw
  ${ROS_INCLUDE_DIR}/rcl_interfaces
)

target_link_directories(ars5_node PUBLIC
  ${ROS_LIB_DIR}
)

target_link_libraries(ars5_node
  rclcpp::rclcpp
  ${ROS_LIB_DIR}/libsensor_msgs__rosidl_typesupport_cpp.so
  ${ROS_LIB_DIR}/libstd_msgs__rosidl_typesupport_cpp.so
  ${ROS_LIB_DIR}/libbuiltin_interfaces__rosidl_typesupport_cpp.so
)

install(TARGETS
  ars5_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
