#ifndef NAV2_OPERATIONS_PLUGINS__CONTROL_ROTOR_HPP_
#define NAV2_OPERATIONS_PLUGINS__CONTROL_ROTOR_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_operations_server/operation.hpp"

namespace nav2_operations_plugins
{

class ControlRotor : public nav2_operations_server::Operation
{
public:
  ControlRotor() = default;
  ~ControlRotor() override = default;

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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr blade_pub_;
  bool blade_state_;
  bool previous_state_;  // For pause/resume
};

}  // namespace nav2_operations_plugins

#endif  // NAV2_OPERATIONS_PLUGINS__CONTROL_ROTOR_HPP_
