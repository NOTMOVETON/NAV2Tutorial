#include <string>
#include <vector>
#include <memory>

#include "nav2_operations_bt_nodes/action/execute_operation_action.hpp"

namespace nav2_operations_bt_nodes
{

ExecuteOperationAction::ExecuteOperationAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_operations_msgs::action::ExecuteOperation>(
    xml_tag_name, action_name, conf)
{
}

void ExecuteOperationAction::on_tick()
{
  std::vector<nav2_operations_msgs::msg::OperationCommand> operations;
  getInput("operations", operations);
  goal_.operations = operations;
}

BT::NodeStatus ExecuteOperationAction::on_success()
{
  setOutput("error_code", result_.result->error_code);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ExecuteOperationAction::on_aborted()
{
  setOutput("error_code", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ExecuteOperationAction::on_cancelled()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_operations_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<nav2_operations_bt_nodes::ExecuteOperationAction>(
        name, "execute_operation", config);
    };
  factory.registerBuilder<nav2_operations_bt_nodes::ExecuteOperationAction>(
    "ExecuteOperation", builder);
}
