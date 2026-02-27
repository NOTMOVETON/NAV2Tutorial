#ifndef NAV2_OPERATIONS_BT_NODES__ACTION__CANCEL_OPERATION_ACTION_HPP_
#define NAV2_OPERATIONS_BT_NODES__ACTION__CANCEL_OPERATION_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_cancel_action_node.hpp"
#include "nav2_operations_msgs/action/execute_operation.hpp"

namespace nav2_operations_bt_nodes
{

class CancelOperationAction
  : public nav2_behavior_tree::BtCancelActionNode<nav2_operations_msgs::action::ExecuteOperation>
{
public:
  CancelOperationAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }
};

}  // namespace nav2_operations_bt_nodes

#endif  // NAV2_OPERATIONS_BT_NODES__ACTION__CANCEL_OPERATION_ACTION_HPP_
