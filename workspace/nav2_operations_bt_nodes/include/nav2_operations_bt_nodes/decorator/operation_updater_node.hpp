#ifndef NAV2_OPERATIONS_BT_NODES__DECORATOR__OPERATION_UPDATER_NODE_HPP_
#define NAV2_OPERATIONS_BT_NODES__DECORATOR__OPERATION_UPDATER_NODE_HPP_

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <utility>

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_operations_msgs/msg/operation_at_pose.hpp"
#include "nav2_operations_msgs/msg/operation_command.hpp"
#include "behaviortree_cpp/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_operations_bt_nodes
{

class OperationUpdater : public BT::DecoratorNode
{
public:
  OperationUpdater(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::InputPort<double>("trigger_distance", 0.1, "Distance threshold for triggering operations"),
      BT::OutputPort<std::vector<nav2_operations_msgs::msg::OperationCommand>>(
        "current_operations",
        "Operations to execute at current trigger"),
    };
  }

private:
  private:
  BT::NodeStatus tick() override;
  void halt() override;

  using OperationCommands = std::vector<nav2_operations_msgs::msg::OperationCommand>;
  using TriggerEntry = std::pair<geometry_msgs::msg::PoseStamped, OperationCommands>;
  using TriggerMap = std::map<int, TriggerEntry>;
  TriggerMap h_;

  std::vector<nav2_operations_msgs::msg::OperationAtPose> current_operations_at_poses_;
  std::vector<nav2_operations_msgs::msg::OperationAtPose> new_operations_at_poses_;

  geometry_msgs::msg::PoseStamped current_pose_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::string global_frame_;
  std::string robot_base_frame_;
  double trigger_distance_;

  bool performing_ = false;
};

}  // namespace nav2_operations_bt_nodes

#endif  // NAV2_OPERATIONS_BT_NODES__DECORATOR__OPERATION_UPDATER_NODE_HPP_