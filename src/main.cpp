#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "sonoptix_echo_driver_ros/sonoptix_echo_driver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sonoptix_echo_driver_ros::SonoptixEchoDriver>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
