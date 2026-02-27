#include <string>
#include <memory>

#include "nav2_operations_bt_nodes/action/resume_operations_server.hpp"

namespace nav2_operations_bt_nodes
{

ResumeOperationsServer::ResumeOperationsServer(
  const std::string & xml_tag_name,
  const std::string & service_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<std_srvs::srv::Trigger>(xml_tag_name, conf, service_name)
{
}

void ResumeOperationsServer::on_tick()
{
  // Trigger service has an empty request, nothing to populate
}

BT::NodeStatus ResumeOperationsServer::on_completion(
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (response->success) {
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "Failed to resume operations server: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace nav2_operations_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<nav2_operations_bt_nodes::ResumeOperationsServer>(
        name, "operations_server/resume", config);
    };
  factory.registerBuilder<nav2_operations_bt_nodes::ResumeOperationsServer>(
    "ResumeOperationsServer", builder);
}
