// Copyright (c) 2024
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

#include <vector>
#include <string>
#include <memory>

#include "nav2_operations_navigator/navigate_with_operations.hpp"

namespace nav2_operations_navigator
{

std::string
NavigateWithOperations::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in getDefaultBTFilepath");
  }

  if (!node->has_parameter("default_follow_path_with_operations_bt_xml")) {
    node->declare_parameter<std::string>(
      "default_follow_path_with_operations_bt_xml", "");
  }

  node->get_parameter("default_follow_path_with_operations_bt_xml", default_bt_xml_filename);
  return default_bt_xml_filename;
}

bool
NavigateWithOperations::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> /*odom_smoother*/)
{
  auto node = parent_node.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in configure");
  }

  // Declare blackboard ID parameters with defaults
  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter<std::string>("path_blackboard_id", "path");
  }
  if (!node->has_parameter("operations_at_poses_blackboard_id")) {
    node->declare_parameter<std::string>(
      "operations_at_poses_blackboard_id", "operations_at_poses");
  }
  if (!node->has_parameter("operations_completed_blackboard_id")) {
    node->declare_parameter<std::string>(
      "operations_completed_blackboard_id", "operations_completed");
  }

  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();
  operations_at_poses_blackboard_id_ =
    node->get_parameter("operations_at_poses_blackboard_id").as_string();
  operations_completed_blackboard_id_ =
    node->get_parameter("operations_completed_blackboard_id").as_string();

  // Set initial blackboard values
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<nav_msgs::msg::Path>(path_blackboard_id_, nav_msgs::msg::Path());
  blackboard->set<std::vector<nav2_operations_msgs::msg::OperationAtPose>>(
    operations_at_poses_blackboard_id_,
    std::vector<nav2_operations_msgs::msg::OperationAtPose>());
  blackboard->set<uint16_t>(operations_completed_blackboard_id_, 0);

  RCLCPP_INFO(logger_, "NavigateWithOperations configured successfully");
  return true;
}

bool
NavigateWithOperations::cleanup()
{
  return true;
}

bool
NavigateWithOperations::goalReceived(
  typename ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;
  RCLCPP_INFO(
      logger_, "Loading tree with path: %s",
      bt_xml_filename.c_str());

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  RCLCPP_INFO(
    logger_,
    "NavigateWithOperations goal received with %zu path poses and %zu operation triggers",
    goal->path.poses.size(),
    goal->operations_at_poses.size());

  auto blackboard = bt_action_server_->getBlackboard();

  // Set blackboard values from goal
  blackboard->set<nav_msgs::msg::Path>(path_blackboard_id_, goal->path);
  blackboard->set<std::vector<nav2_operations_msgs::msg::OperationAtPose>>(
    operations_at_poses_blackboard_id_, goal->operations_at_poses);
  blackboard->set<uint16_t>(operations_completed_blackboard_id_, 0);

  active_goal_ = true;
  return true;
}

void
NavigateWithOperations::onLoop()
{
  auto feedback_msg = std::make_shared<ActionT::Feedback>();
  auto blackboard = bt_action_server_->getBlackboard();

  // Read operations_completed from blackboard
  uint16_t operations_completed = 0;
  auto res = blackboard->get<uint16_t>(operations_completed_blackboard_id_, operations_completed);
  feedback_msg->operations_completed = operations_completed;

  // Publish feedback
  bt_action_server_->publishFeedback(feedback_msg);
}

void
NavigateWithOperations::goalCompleted(
  typename ActionT::Result::SharedPtr result,
  const nav2_behavior_tree::BtStatus final_bt_status)
{
  active_goal_ = false;

  if (final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED) {
    RCLCPP_INFO(logger_, "NavigateWithOperations completed successfully");
    result->error_code = 0; // stub
  } else if (final_bt_status == nav2_behavior_tree::BtStatus::FAILED) {
    RCLCPP_WARN(logger_, "NavigateWithOperations failed");
    result->error_code = 1; // stub
  } else if (final_bt_status == nav2_behavior_tree::BtStatus::CANCELED) {
    RCLCPP_INFO(logger_, "NavigateWithOperations cancelled");
    result->error_code = 2; // stub
  }

  // Clean blackboard fields
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<nav_msgs::msg::Path>(path_blackboard_id_, nav_msgs::msg::Path());
  blackboard->set<std::vector<nav2_operations_msgs::msg::OperationAtPose>>(
    operations_at_poses_blackboard_id_,
    std::vector<nav2_operations_msgs::msg::OperationAtPose>());
  blackboard->set<uint16_t>(operations_completed_blackboard_id_, 0);
}

void
NavigateWithOperations::onPreempt(
  typename ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    // Accept the pending goal if it uses the same BT or default BT
    auto pending_goal = bt_action_server_->acceptPendingGoal();
    auto blackboard = bt_action_server_->getBlackboard();

    // Update blackboard with new goal values
    blackboard->set<nav_msgs::msg::Path>(path_blackboard_id_, pending_goal->path);
    blackboard->set<std::vector<nav2_operations_msgs::msg::OperationAtPose>>(
      operations_at_poses_blackboard_id_, pending_goal->operations_at_poses);
    blackboard->set<uint16_t>(operations_completed_blackboard_id_, 0);
  } else {
    RCLCPP_WARN(
      logger_,
      "Preemption request was rejected since the requested BT XML file is not the same "
      "as the one that the current goal is executing. Preemption with a new BT is invalid "
      "since it would require cancellation of the previous goal instead of true preemption."
      "\nCancel the current goal and send a new action request if you want to use a "
      "different BT XML file. For now, continuing to track the last goal until completion.");
    bt_action_server_->terminatePendingGoal();
  }
}

}  // namespace nav2_operations_navigator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_operations_navigator::NavigateWithOperations,
  nav2_core::NavigatorBase)
