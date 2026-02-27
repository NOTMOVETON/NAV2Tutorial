#ifndef NAV2_OPERATIONS_SERVER__OPERATIONS_SERVER_HPP_
#define NAV2_OPERATIONS_SERVER__OPERATIONS_SERVER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav2_operations_server/operation.hpp"
#include "nav2_operations_msgs/action/execute_operation.hpp"

namespace nav2_operations_server
{

class OperationsServer : public nav2_util::LifecycleNode
{
public:
  using ExecuteOperation = nav2_operations_msgs::action::ExecuteOperation;
  using GoalHandleExecuteOperation = rclcpp_action::ServerGoalHandle<ExecuteOperation>;
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit OperationsServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OperationsServer() override;

protected:
  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteOperation::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteOperation> goal_handle);
  void handle_accepted(
    const std::shared_ptr<GoalHandleExecuteOperation> goal_handle);
  void execute(const std::shared_ptr<GoalHandleExecuteOperation> goal_handle);

  // Service callbacks
  void pause_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void resume_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Plugin management
  pluginlib::ClassLoader<Operation> plugin_loader_;
  std::map<std::string, Operation::Ptr> operations_;
  std::map<std::string, bool> instant_flags_;
  std::vector<std::string> plugin_names_;

  // Action server
  rclcpp_action::Server<ExecuteOperation>::SharedPtr action_server_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;

  // Parameters
  double operation_frequency_;
  bool paused_;
};

}  // namespace nav2_operations_server

#endif  // NAV2_OPERATIONS_SERVER__OPERATIONS_SERVER_HPP_
