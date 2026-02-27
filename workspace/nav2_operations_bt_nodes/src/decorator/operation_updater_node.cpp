#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

#include "nav2_operations_bt_nodes/decorator/operation_updater_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_operations_bt_nodes
{

OperationUpdater::OperationUpdater(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  initialized_(false),
  marker_id_(0)
{
}

BT::NodeStatus OperationUpdater::tick()
{
  if (!initialized_) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/operation_events", rclcpp::QoS(10));

    // Read operations_at_poses from blackboard (not a port)
    remaining_operations_ =
      config().blackboard->get<std::vector<nav2_operations_msgs::msg::OperationAtPose>>(
      "operations_at_poses");

    getInput("global_frame", global_frame_);
    getInput("robot_base_frame", robot_base_frame_);
    getInput("trigger_distance", trigger_distance_);

    initialized_ = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "OperationUpdater initialized with %zu operation poses, trigger_distance=%.2f",
      remaining_operations_.size(), trigger_distance_);
  }

  // Get current robot pose
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) {
    RCLCPP_WARN(node_->get_logger(), "OperationUpdater: Failed to get robot pose");
    return BT::NodeStatus::FAILURE;
  }

  // Check each remaining operation trigger pose
  for (auto it = remaining_operations_.begin(); it != remaining_operations_.end(); ++it) {
    geometry_msgs::msg::PoseStamped trigger_pose;
    trigger_pose.header = it->pose.header;
    trigger_pose.pose = it->pose.pose;

    double dist = poseDistance(robot_pose, trigger_pose);
    if (dist < trigger_distance_) {
      // Robot is within trigger distance of this operation pose
      RCLCPP_INFO(
        node_->get_logger(),
        "OperationUpdater: Triggered at distance %.2f (threshold %.2f), %zu operations",
        dist, trigger_distance_, it->operations.size());

      // Set the current_operations output port
      setOutput("current_operations", it->operations);

      // Publish trigger marker
      if (!it->operations.empty()) {
        publishTriggerMarker(trigger_pose, it->operations[0].action_name);
      }

      // Remove this trigger from remaining
      remaining_operations_.erase(it);

      // Increment operations_completed on blackboard
      uint16_t count = 0;
      auto res = config().blackboard->get("operations_completed", count);
      config().blackboard->set("operations_completed", static_cast<uint16_t>(count + 1));

      // Tick the child node (e.g., ExecuteOperation)
      auto child_status = child_node_->executeTick();
      return child_status;
    }
  }

  // No trigger nearby — return SUCCESS to let parent (e.g., ReactiveSequence) continue
  return BT::NodeStatus::SUCCESS;
}

bool OperationUpdater::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero);

    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(),
      "OperationUpdater: Could not get robot pose: %s", ex.what());
    return false;
  }
}

double OperationUpdater::poseDistance(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  double dx = a.pose.position.x - b.pose.position.x;
  double dy = a.pose.position.y - b.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

void OperationUpdater::publishTriggerMarker(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & action_name)
{
  visualization_msgs::msg::Marker marker;
  marker.header = pose.header;
  marker.header.stamp = node_->now();
  marker.ns = "operation_triggers";
  marker.id = marker_id_++;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose.pose;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.8;
  marker.lifetime = rclcpp::Duration::from_seconds(10.0);
  marker.text = action_name;

  marker_pub_->publish(marker);
}

}  // namespace nav2_operations_bt_nodes

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_operations_bt_nodes::OperationUpdater>("OperationUpdater");
}
