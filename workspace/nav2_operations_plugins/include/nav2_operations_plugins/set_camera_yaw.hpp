#ifndef NAV2_OPERATIONS_PLUGINS__SET_CAMERA_YAW_HPP_
#define NAV2_OPERATIONS_PLUGINS__SET_CAMERA_YAW_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav2_operations_server/operation.hpp"

namespace nav2_operations_plugins
{

class SetCameraYaw : public nav2_operations_server::Operation
{
public:
  SetCameraYaw() = default;
  ~SetCameraYaw() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;

  void setGoal(const nav2_operations_msgs::msg::OperationCommand & command) override;
  void execute() override;
  bool isDone() override;
  void stop() override;
  void pause() override;
  void resume() override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr camera_pub_;
  double current_yaw_;
  double target_yaw_;
  double max_rotation_speed_;  // degrees per second
  double tolerance_;           // degrees
  bool rotating_;
  rclcpp::Time last_execute_time_;
};

}  // namespace nav2_operations_plugins

#endif  // NAV2_OPERATIONS_PLUGINS__SET_CAMERA_YAW_HPP_
