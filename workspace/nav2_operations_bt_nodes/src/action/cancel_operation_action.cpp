#include <string>
#include <memory>

#include "nav2_operations_bt_nodes/action/cancel_operation_action.hpp"

namespace nav2_operations_bt_nodes
{

CancelOperationAction::CancelOperationAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtCancelActionNode<nav2_operations_msgs::action::ExecuteOperation>(
    xml_tag_name, action_name, conf)
{
}

}  // namespace nav2_operations_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<nav2_operations_bt_nodes::CancelOperationAction>(
        name, "execute_operation", config);
    };
  factory.registerBuilder<nav2_operations_bt_nodes::CancelOperationAction>(
    "CancelOperation", builder);
}
