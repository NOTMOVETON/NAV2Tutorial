#include <cmath>
#include "nav2_operations_plugins/set_camera_yaw.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_operations_plugins
{

void SetCameraYaw::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name)
{
  node_ = parent;
  name_ = name;
  auto node = node_.lock();

  camera_pub_ = node->create_publisher<std_msgs::msg::Float32>(
    "/lawnmower/camera_yaw_command", rclcpp::QoS(1));

  node->declare_parameter(name_ + ".max_rotation_speed", 45.0);
  node->get_parameter(name_ + ".max_rotation_speed", max_rotation_speed_);

  current_yaw_ = 0.0;
  target_yaw_ = 0.0;
  tolerance_ = 1.0;
  rotating_ = false;

  RCLCPP_INFO(
    node->get_logger(),
    "[SetCameraYaw] Configured with max_rotation_speed: %.1f deg/s",
    max_rotation_speed_);
}

void SetCameraYaw::activate()
{
}

void SetCameraYaw::deactivate()
{
}

void SetCameraYaw::cleanup()
{
  camera_pub_.reset();
}

void SetCameraYaw::setGoal(
  const nav2_operations_msgs::msg::OperationCommand & command)
{
  if (!command.float_states.empty()) {
    target_yaw_ = static_cast<double>(command.float_states[0]);
  } else {
    target_yaw_ = 0.0;
  }
  rotating_ = true;

  auto node = node_.lock();
  if (node) {
    last_execute_time_ = node->now();
  }
}

void SetCameraYaw::execute()
{
  if (!rotating_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    return;
  }

  rclcpp::Time now = node->now();
  double dt = (now - last_execute_time_).seconds();
  last_execute_time_ = now;

  double step = max_rotation_speed_ * dt;
  double diff = target_yaw_ - current_yaw_;

  if (std::abs(diff) <= step) {
    current_yaw_ = target_yaw_;
  } else if (diff > 0.0) {
    current_yaw_ += step;
  } else {
    current_yaw_ -= step;
  }

  auto msg = std_msgs::msg::Float32();
  msg.data = static_cast<float>(current_yaw_);
  camera_pub_->publish(msg);

  RCLCPP_INFO(
    node->get_logger(),
    "[SetCameraYaw] Rotating to %.1f deg, current: %.1f deg",
    target_yaw_, current_yaw_);
}

bool SetCameraYaw::isDone()
{
  return std::abs(current_yaw_ - target_yaw_) < tolerance_;
}

void SetCameraYaw::stop()
{
  rotating_ = false;
}

void SetCameraYaw::pause()
{
  rotating_ = false;
}

void SetCameraYaw::resume()
{
  rotating_ = true;
  auto node = node_.lock();
  if (node) {
    last_execute_time_ = node->now();
  }
}

}  // namespace nav2_operations_plugins

PLUGINLIB_EXPORT_CLASS(
  nav2_operations_plugins::SetCameraYaw,
  nav2_operations_server::Operation)
