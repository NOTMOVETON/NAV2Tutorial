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

#ifndef NAV2_OPERATIONS_NAVIGATOR__NAVIGATE_WITH_OPERATIONS_HPP_
#define NAV2_OPERATIONS_NAVIGATOR__NAVIGATE_WITH_OPERATIONS_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_core/behavior_tree_navigator.hpp"
#include "nav2_operations_msgs/action/follow_path_with_operations.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_operations_msgs/msg/operation_at_pose.hpp"
#include "nav2_util/odometry_utils.hpp"

namespace nav2_operations_navigator
{

/**
 * @class NavigateWithOperations
 * @brief A navigator plugin for following a path with operations at poses.
 * Derives from BehaviorTreeNavigator and is loaded by bt_navigator as a pluginlib plugin.
 */
class NavigateWithOperations
  : public nav2_core::BehaviorTreeNavigator<
      nav2_operations_msgs::action::FollowPathWithOperations>
{
public:
  using ActionT = nav2_operations_msgs::action::FollowPathWithOperations;

  /**
   * @brief Constructor
   */
  NavigateWithOperations()
  : BehaviorTreeNavigator() {}

  /**
   * @brief Get the action name for this navigator
   * @return string Name of the action server
   */
  std::string getName() override {return std::string("navigate_with_operations");}

  /**
   * @brief Get the default BT filename parameter for this navigator
   * @param node WeakPtr to the lifecycle node
   * @return string Filepath to the default BT XML
   */
  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

  /**
   * @brief Configure the navigator on bt_navigator configure
   * @param node WeakPtr to the lifecycle node
   * @param odom_smoother Object to get current smoothed robot speed
   * @return bool If successful
   */
  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  /**
   * @brief Cleanup on bt_navigator cleanup
   * @return bool If successful
   */
  bool cleanup() override;

  /**
   * @brief Pause the navigator (no-op)
   */
  void pause() {}

  /**
   * @brief Resume the navigator (no-op)
   */
  void resume() {}

  /**
   * @brief Check if there is an active goal
   * @return bool True if a goal is being processed
   */
  bool has_active_goal() { return active_goal_; }

protected:
  /**
   * @brief Called when a new goal is received
   * @param goal Action goal message
   * @return bool If goal was accepted for processing
   */
  bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief Called on each BT loop iteration to publish feedback
   */
  void onLoop() override;

  /**
   * @brief Called when the goal navigation is completed
   * @param result Action result message to populate
   * @param final_bt_status Resulting status of the BT execution
   */
  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

  /**
   * @brief Called when a preemption is requested (new goal while processing)
   * @param goal The new preempting goal
   */
  void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) override;

  // Blackboard variable names
  std::string path_blackboard_id_;
  std::string operations_at_poses_blackboard_id_;
  std::string operations_completed_blackboard_id_;

  bool active_goal_{false};
};

}  // namespace nav2_operations_navigator

#endif  // NAV2_OPERATIONS_NAVIGATOR__NAVIGATE_WITH_OPERATIONS_HPP_
