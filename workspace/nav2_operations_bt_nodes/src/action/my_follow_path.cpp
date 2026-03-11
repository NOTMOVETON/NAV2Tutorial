// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>

#include "nav2_operations_bt_nodes/action/my_follow_path.hpp"

namespace nav2_operations_bt_nodes
{

MyFollowPathAction::MyFollowPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void MyFollowPathAction::on_tick()
{
  getInput("path", goal_.path);
  getInput("controller_id", goal_.controller_id);
  getInput("goal_checker_id", goal_.goal_checker_id);
  getInput("progress_checker_id", goal_.progress_checker_id);

  nav_msgs::msg::Path current_path;
  if (config().blackboard->get("path", current_path)) {
    RCLCPP_INFO(node_->get_logger(),
      "OperationUpdater returning SUCCESS, path has %zu poses",
      current_path.poses.size());
  } else{
    RCLCPP_INFO(node_->get_logger(),
      "Cant get path");
  }
  callback_group_executor_.spin_some();
}

BT::NodeStatus MyFollowPathAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MyFollowPathAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MyFollowPathAction::on_cancelled()
{
  // Set empty error code, action was cancelled
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

void MyFollowPathAction::on_wait_for_result(
  std::shared_ptr<const Action::Feedback>/*feedback*/)
{
  // Grab the new path
  nav_msgs::msg::Path new_path;
  getInput("path", new_path);

  nav_msgs::msg::Path current_path;
  if (config().blackboard->get("path", current_path)) {
    RCLCPP_INFO(node_->get_logger(),
      "OperationUpdater returning SUCCESS, path has %zu poses",
      current_path.poses.size());
  } else{
    RCLCPP_INFO(node_->get_logger(),
      "Cant get path");
  }
  callback_group_executor_.spin_some();

  // Check if it is not same with the current one
  if (goal_.path != new_path && new_path != nav_msgs::msg::Path()) {
    // the action server on the next loop iteration
    goal_.path = new_path;
    goal_updated_ = true;
  }

  std::string new_controller_id;
  getInput("controller_id", new_controller_id);

  if (goal_.controller_id != new_controller_id) {
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }

  std::string new_goal_checker_id;
  getInput("goal_checker_id", new_goal_checker_id);

  if (goal_.goal_checker_id != new_goal_checker_id) {
    goal_.goal_checker_id = new_goal_checker_id;
    goal_updated_ = true;
  }

  std::string new_progress_checker_id;
  getInput("progress_checker_id", new_progress_checker_id);

  if (goal_.progress_checker_id != new_progress_checker_id) {
    goal_.progress_checker_id = new_progress_checker_id;
    goal_updated_ = true;
  }
}

}  // namespace nav2_operations_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_operations_bt_nodes::MyFollowPathAction>(
        name, "follow_path", config);
    };

  factory.registerBuilder<nav2_operations_bt_nodes::MyFollowPathAction>(
    "MyFollowPath", builder);
}