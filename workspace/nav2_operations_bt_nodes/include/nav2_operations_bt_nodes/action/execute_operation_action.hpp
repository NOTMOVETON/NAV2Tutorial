#ifndef NAV2_OPERATIONS_BT_NODES__ACTION__EXECUTE_OPERATION_ACTION_HPP_
#define NAV2_OPERATIONS_BT_NODES__ACTION__EXECUTE_OPERATION_ACTION_HPP_

#include <string>
#include <vector>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_operations_msgs/action/execute_operation.hpp"
#include "nav2_operations_msgs/msg/operation_command.hpp"
#include "nav2_operations_bt_nodes/bt_conversions.hpp"

namespace nav2_operations_bt_nodes
{

class ExecuteOperationAction
  : public nav2_behavior_tree::BtActionNode<nav2_operations_msgs::action::ExecuteOperation>
{
  using Action = nav2_operations_msgs::action::ExecuteOperation;
  using ActionGoal = Action::Goal;

public:
  ExecuteOperationAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::vector<nav2_operations_msgs::msg::OperationCommand>>(
        "operations", "Operations to execute"),
      BT::OutputPort<uint16_t>("error_code", "Error code from result"),
    });
  }

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
};

}  // namespace nav2_operations_bt_nodes

#endif  // NAV2_OPERATIONS_BT_NODES__ACTION__EXECUTE_OPERATION_ACTION_HPP_
