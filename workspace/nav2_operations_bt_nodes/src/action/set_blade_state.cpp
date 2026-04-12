// Copyright (c) 2024 Apache-2.0
#include "nav2_operations_bt_nodes/action/set_blade_state.hpp"

namespace nav2_operations_bt_nodes
{

void SetBladeState::on_tick()
{
  bool enable = true;
  getInput("enable", enable);
  goal_.enable = enable;
}

}  // namespace nav2_operations_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<nav2_operations_bt_nodes::SetBladeState>(
        name, "blade_server", config);
    };
  factory.registerBuilder<nav2_operations_bt_nodes::SetBladeState>(
    "SetBladeState", builder);
}
