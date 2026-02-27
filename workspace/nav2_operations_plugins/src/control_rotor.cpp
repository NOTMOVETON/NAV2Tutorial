#include "nav2_operations_plugins/control_rotor.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_operations_plugins
{

void ControlRotor::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name)
{
  node_ = parent;
  name_ = name;
  auto node = node_.lock();
  blade_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "/lawnmower/blade_command", rclcpp::QoS(1));
  blade_state_ = false;
  previous_state_ = false;
  RCLCPP_INFO(node->get_logger(), "[ControlRotor] Configured");
}

void ControlRotor::activate()
{
}

void ControlRotor::deactivate()
{
}

void ControlRotor::cleanup()
{
  blade_pub_.reset();
}

void ControlRotor::setGoal(
  const nav2_operations_msgs::msg::OperationCommand & command)
{
  if (!command.bool_states.empty()) {
    blade_state_ = command.bool_states[0];
  } else {
    blade_state_ = false;
  }
}

void ControlRotor::execute()
{
  auto msg = std_msgs::msg::Bool();
  msg.data = blade_state_;
  blade_pub_->publish(msg);

  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(
      node->get_logger(), "[ControlRotor] Blade turned %s",
      blade_state_ ? "ON" : "OFF");
  }
}

bool ControlRotor::isDone()
{
  return true;
}

void ControlRotor::stop()
{
  blade_state_ = false;
  auto msg = std_msgs::msg::Bool();
  msg.data = blade_state_;
  blade_pub_->publish(msg);
}

void ControlRotor::pause()
{
  previous_state_ = blade_state_;
  blade_state_ = false;
  auto msg = std_msgs::msg::Bool();
  msg.data = blade_state_;
  blade_pub_->publish(msg);
}

void ControlRotor::resume()
{
  blade_state_ = previous_state_;
  auto msg = std_msgs::msg::Bool();
  msg.data = blade_state_;
  blade_pub_->publish(msg);
}

}  // namespace nav2_operations_plugins

PLUGINLIB_EXPORT_CLASS(
  nav2_operations_plugins::ControlRotor,
  nav2_operations_server::Operation)
