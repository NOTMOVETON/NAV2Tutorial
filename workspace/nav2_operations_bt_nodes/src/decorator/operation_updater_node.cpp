#include <string>
#include <memory>
#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp/decorator_node.h"
#include "nav2_operations_bt_nodes/decorator/operation_updater_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_operations_bt_nodes
{

OperationUpdater::OperationUpdater(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  getInput("trigger_distance", trigger_distance_);
}

BT::NodeStatus OperationUpdater::tick()
{
  callback_group_executor_.spin_some();

  new_operations_at_poses_ =
    config().blackboard->get<std::vector<nav2_operations_msgs::msg::OperationAtPose>>(
    "operations_at_poses");

  if (current_operations_at_poses_ != new_operations_at_poses_) {
    RCLCPP_INFO(
      node_->get_logger(), "OperationUpdater: setting new operations list (%zu poses)",
      new_operations_at_poses_.size());

    performing_ = false;
    current_operations_at_poses_ = new_operations_at_poses_;
    OperationUpdater::haltChild();

    h_.clear();
    for (int i = 0; i < static_cast<int>(current_operations_at_poses_.size()); ++i) {
      h_[i] = {current_operations_at_poses_[i].pose, current_operations_at_poses_[i].operations};
    }
    RCLCPP_INFO(node_->get_logger(), "OperationUpdater: operations_size: %d", int(h_.size()));
  }

  if (!nav2_util::getCurrentPose(current_pose_, *tf_, global_frame_, robot_base_frame_, 0.2)) {
    RCLCPP_WARN(node_->get_logger(), "OperationUpdater: failed to get robot pose");
    return BT::NodeStatus::FAILURE;
  }

  if (performing_) {
    RCLCPP_INFO(node_->get_logger(), "OperationUpdater: PERFORMING");
    const BT::NodeStatus child_state = child_node_->executeTick();
    if (child_state != BT::NodeStatus::RUNNING) {
      performing_ = false;
      RCLCPP_INFO(node_->get_logger(), "OperationUpdater: STOP PERFORMING");
      return child_state;  // propagate SUCCESS or FAILURE to the tree
    }
    return BT::NodeStatus::RUNNING;
  }

  for (auto & [idx, entry] : h_) {
    const auto & [trigger_pose, operations] = entry;
    double dx = current_pose_.pose.position.x - trigger_pose.pose.position.x;
    double dy = current_pose_.pose.position.y - trigger_pose.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < trigger_distance_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "OperationUpdater: triggered at index %d, distance %.2f, %zu operations",
        idx, dist, operations.size());

      setOutput("current_operations", operations);
      h_.erase(idx);

      uint16_t count = 0;
      [[maybe_unused]] auto res = config().blackboard->get("operations_completed", count);
      config().blackboard->set("operations_completed", static_cast<uint16_t>(count + 1));

      const BT::NodeStatus child_state = child_node_->executeTick();
      if (child_state == BT::NodeStatus::RUNNING) {
        RCLCPP_INFO(node_->get_logger(), "OperationUpdater: PERFORMING");
        performing_ = true;
      }
      return child_state;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

void OperationUpdater::halt()
{
  performing_ = false;
  haltChild();
  BT::DecoratorNode::halt();
}

}  // namespace nav2_operations_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_operations_bt_nodes::OperationUpdater>("OperationUpdater");
}