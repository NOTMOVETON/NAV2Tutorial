#ifndef NAV2_OPERATIONS_BT_NODES__ACTION__RESUME_OPERATIONS_SERVER_HPP_
#define NAV2_OPERATIONS_BT_NODES__ACTION__RESUME_OPERATIONS_SERVER_HPP_

#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace nav2_operations_bt_nodes
{

class ResumeOperationsServer
  : public nav2_behavior_tree::BtServiceNode<std_srvs::srv::Trigger>
{
public:
  ResumeOperationsServer(
    const std::string & xml_tag_name,
    const std::string & service_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  void on_tick() override;
  BT::NodeStatus on_completion(
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) override;
};

}  // namespace nav2_operations_bt_nodes

#endif  // NAV2_OPERATIONS_BT_NODES__ACTION__RESUME_OPERATIONS_SERVER_HPP_
