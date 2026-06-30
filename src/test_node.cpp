#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("ars5_test_node");
  RCLCPP_INFO(node->get_logger(), "ARS5 test node started");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
