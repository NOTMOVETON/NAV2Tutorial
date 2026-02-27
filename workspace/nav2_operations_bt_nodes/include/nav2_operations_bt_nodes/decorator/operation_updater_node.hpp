#ifndef NAV2_OPERATIONS_BT_NODES__DECORATOR__OPERATION_UPDATER_NODE_HPP_
#define NAV2_OPERATIONS_BT_NODES__DECORATOR__OPERATION_UPDATER_NODE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "behaviortree_cpp/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav2_operations_msgs/msg/operation_at_pose.hpp"
#include "nav2_operations_msgs/msg/operation_command.hpp"

namespace nav2_operations_bt_nodes
{

class OperationUpdater : public BT::DecoratorNode
{
public:
  OperationUpdater(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("global_frame", "map", "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", "base_link", "Robot base frame"),
      BT::InputPort<double>("trigger_distance", 1.0, "Distance threshold for triggering operations"),
      BT::OutputPort<std::vector<nav2_operations_msgs::msg::OperationCommand>>(
        "current_operations", "Operations to execute at current trigger"),
    };
  }

  BT::NodeStatus tick() override;

private:
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);
  double poseDistance(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b);
  void publishTriggerMarker(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & action_name);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::vector<nav2_operations_msgs::msg::OperationAtPose> remaining_operations_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double trigger_distance_;
  bool initialized_;
  int marker_id_;
};

}  // namespace nav2_operations_bt_nodes

#endif  // NAV2_OPERATIONS_BT_NODES__DECORATOR__OPERATION_UPDATER_NODE_HPP_
