#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_operations_server/operations_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_operations_server::OperationsServer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
