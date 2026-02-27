#ifndef NAV2_OPERATIONS_SERVER__OPERATION_HPP_
#define NAV2_OPERATIONS_SERVER__OPERATION_HPP_

#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_operations_msgs/msg/operation_command.hpp"

namespace nav2_operations_server
{

class Operation
{
public:
  using Ptr = std::shared_ptr<Operation>;
  virtual ~Operation() = default;

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name) = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;
  virtual void cleanup() = 0;

  virtual void setGoal(
    const nav2_operations_msgs::msg::OperationCommand & command) = 0;
  virtual void execute() = 0;
  virtual bool isDone() = 0;
  virtual void stop() = 0;
  virtual void pause() = 0;
  virtual void resume() = 0;
};

}  // namespace nav2_operations_server

#endif  // NAV2_OPERATIONS_SERVER__OPERATION_HPP_
