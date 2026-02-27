#ifndef NAV2_OPERATIONS_SERVER__OPERATIONS_SERVER_HPP_
#define NAV2_OPERATIONS_SERVER__OPERATIONS_SERVER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
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
  using ActionServer = nav2_util::SimpleActionServer<ExecuteOperation>;
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

  // Action server callback
  void executeOperation();

  // Service callbacks
  void pause_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void resume_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Dynamic parameters callback
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Plugin management
  pluginlib::ClassLoader<Operation> plugin_loader_;
  std::map<std::string, Operation::Ptr> operations_;
  std::map<std::string, bool> instant_flags_;
  
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> operation_ids_;
  std::vector<std::string> operation_types_;
  std::string operation_ids_concat_;

  // Action server
  std::unique_ptr<ActionServer> action_server_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  // Parameters
  double operation_frequency_;
  double action_server_result_timeout_;
  bool paused_;
};

}  // namespace nav2_operations_server

#endif  // NAV2_OPERATIONS_SERVER__OPERATIONS_SERVER_HPP_
