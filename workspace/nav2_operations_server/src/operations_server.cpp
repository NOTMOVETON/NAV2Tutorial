#include "nav2_operations_server/operations_server.hpp"

#include <chrono>
#include <functional>
#include <thread>
#include <utility>

#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using std::placeholders::_2;

namespace nav2_operations_server
{

OperationsServer::OperationsServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("operations_server", "", options),
  plugin_loader_("nav2_operations_server", "nav2_operations_server::Operation"),
  default_ids_{"ControlRotor", "SetCameraYaw"},
  default_types_{"nav2_operations_plugins::ControlRotor", "nav2_operations_plugins::SetCameraYaw"},
  paused_(false)
{
  RCLCPP_INFO(get_logger(), "Creating OperationsServer");

  declare_parameter("operation_frequency", 10.0);
  declare_parameter("action_server_result_timeout", 10.0);
  declare_parameter("operation_plugins", default_ids_);
}

OperationsServer::~OperationsServer()
{
  RCLCPP_INFO(get_logger(), "Destroying OperationsServer");
  operations_.clear();
}

OperationsServer::CallbackReturn
OperationsServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring OperationsServer");

  get_parameter("operation_plugins", operation_ids_);
  if (operation_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
  }

  operation_types_.resize(operation_ids_.size());

  get_parameter("operation_frequency", operation_frequency_);
  RCLCPP_INFO(get_logger(), "Operation frequency set to %.4fHz", operation_frequency_);

  for (size_t i = 0; i != operation_ids_.size(); i++) {
    try {
      operation_types_[i] = nav2_util::get_plugin_type_param(node, operation_ids_[i]);
      Operation::Ptr operation = plugin_loader_.createUniqueInstance(operation_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created operation : %s of type %s",
        operation_ids_[i].c_str(), operation_types_[i].c_str());
      operation->configure(weak_from_this(), operation_ids_[i]);
      
      // We need to get action_name and instant flag for each plugin
      nav2_util::declare_parameter_if_not_declared(
        node, operation_ids_[i] + ".action_name", rclcpp::ParameterValue(operation_ids_[i]));
      nav2_util::declare_parameter_if_not_declared(
        node, operation_ids_[i] + ".instant", rclcpp::ParameterValue(false));

      std::string action_name;
      bool instant = false;
      get_parameter(operation_ids_[i] + ".action_name", action_name);
      get_parameter(operation_ids_[i] + ".instant", instant);

      operations_.insert({action_name, operation});
      instant_flags_.insert({action_name, instant});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create operation. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != operation_ids_.size(); i++) {
    operation_ids_concat_ += operation_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Operations Server has %s operations available.", operation_ids_concat_.c_str());

  // Create pause/resume services
  pause_service_ = create_service<std_srvs::srv::Trigger>(
    "operations_server/pause",
    std::bind(&OperationsServer::pause_callback, this, _1, _2));

  resume_service_ = create_service<std_srvs::srv::Trigger>(
    "operations_server/resume",
    std::bind(&OperationsServer::resume_callback, this, _1, _2));

  double action_server_result_timeout;
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  // Create action server
  try {
    action_server_ = std::make_unique<ActionServer>(
      shared_from_this(),
      "execute_operation",
      std::bind(&OperationsServer::executeOperation, this),
      nullptr,
      std::chrono::milliseconds(500),
      true /*spin thread*/, server_options);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(get_logger(), "Error creating action server! %s", e.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "OperationsServer configured successfully");
  return CallbackReturn::SUCCESS;
}

OperationsServer::CallbackReturn
OperationsServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating OperationsServer");

  for (auto & [action_name, plugin] : operations_) {
    plugin->activate();
  }

  action_server_->activate();

  auto node = shared_from_this();
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&OperationsServer::dynamicParametersCallback, this, _1));

  // Register bond with lifecycle manager so it knows this node is alive
  createBond();

  RCLCPP_INFO(get_logger(), "OperationsServer activated");
  return CallbackReturn::SUCCESS;
}

OperationsServer::CallbackReturn
OperationsServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating OperationsServer");

  action_server_->deactivate();

  for (auto & [action_name, plugin] : operations_) {
    plugin->deactivate();
  }

  remove_on_set_parameters_callback(dyn_params_handler_.get());
  dyn_params_handler_.reset();

  // Destroy bond with lifecycle manager
  destroyBond();

  RCLCPP_INFO(get_logger(), "OperationsServer deactivated");
  return CallbackReturn::SUCCESS;
}

OperationsServer::CallbackReturn
OperationsServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up OperationsServer");

  for (auto & [action_name, plugin] : operations_) {
    plugin->cleanup();
  }

  operations_.clear();
  instant_flags_.clear();
  operation_ids_.clear();
  operation_types_.clear();
  operation_ids_concat_.clear();

  action_server_.reset();
  pause_service_.reset();
  resume_service_.reset();

  RCLCPP_INFO(get_logger(), "OperationsServer cleaned up");
  return CallbackReturn::SUCCESS;
}

OperationsServer::CallbackReturn
OperationsServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down OperationsServer");
  return CallbackReturn::SUCCESS;
}

void
OperationsServer::executeOperation()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  RCLCPP_INFO(get_logger(), "Executing operation goal");

  auto goal = action_server_->get_current_goal();
  if (!goal) {
    return;
  }

  auto result = std::make_shared<ExecuteOperation::Result>();
  auto feedback = std::make_shared<ExecuteOperation::Feedback>();

  result->operations_total = static_cast<uint16_t>(goal->operations.size());
  feedback->operations_total = static_cast<uint16_t>(goal->operations.size());
  uint16_t operations_completed = 0;

  rclcpp::WallRate loop_rate(operation_frequency_);

  for (const auto & operation : goal->operations) {
    const auto & action_name = operation.action_name;

    // Find the plugin for this action
    auto it = operations_.find(action_name);
    if (it == operations_.end()) {
      RCLCPP_ERROR(
        get_logger(), "Unknown operation: '%s'", action_name.c_str());
      result->error_code = ExecuteOperation::Result::UNKNOWN_OPERATION;
      result->error_msg = "Unknown operation: " + action_name;
      result->operations_completed = operations_completed;
      action_server_->terminate_current(result);
      return;
    }

    // Wait while paused before starting the operation
    while (paused_ && rclcpp::ok()) {
      if (action_server_->is_cancel_requested()) {
        result->error_code = ExecuteOperation::Result::CANCELLED;
        result->error_msg = "Cancelled while paused";
        result->operations_completed = operations_completed;
        action_server_->terminate_all(result);
        return;
      }
      std::this_thread::sleep_for(100ms);
    }

    auto & plugin = it->second;
    bool instant = instant_flags_[action_name];

    // Set the goal on the plugin
    plugin->setGoal(operation);

    // Publish feedback
    feedback->current_operation_name = action_name;
    feedback->operations_completed = operations_completed;
    action_server_->publish_feedback(feedback);

    RCLCPP_INFO(
      get_logger(), "Executing operation '%s' (instant: %s)",
      action_name.c_str(), instant ? "true" : "false");

    if (instant) {
      // Instant operation: execute once
      plugin->execute();
      operations_completed++;
    } else {
      // Continuous operation: loop at operation_frequency_
      while (rclcpp::ok()) {
        if (action_server_ == nullptr || !action_server_->is_server_active()) {
          RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
          return;
        }

        // Check for cancellation
        if (action_server_->is_cancel_requested()) {
          RCLCPP_INFO(get_logger(), "Operation '%s' cancelled", action_name.c_str());
          plugin->stop();
          result->error_code = ExecuteOperation::Result::CANCELLED;
          result->error_msg = "Operation cancelled";
          result->operations_completed = operations_completed;
          action_server_->terminate_all(result);
          return;
        }

        // Handle pause
        if (paused_) {
          plugin->pause();
          while (paused_ && rclcpp::ok()) {
            if (action_server_->is_cancel_requested()) {
              plugin->stop();
              result->error_code = ExecuteOperation::Result::CANCELLED;
              result->error_msg = "Cancelled while paused";
              result->operations_completed = operations_completed;
              action_server_->terminate_all(result);
              return;
            }
            std::this_thread::sleep_for(100ms);
          }
          plugin->resume();
        }

        plugin->execute();

        if (plugin->isDone()) {
          RCLCPP_INFO(get_logger(), "Operation '%s' completed", action_name.c_str());
          break;
        }

        loop_rate.sleep();
      }
      operations_completed++;
    }
  }

  // Set final result
  result->error_code = ExecuteOperation::Result::NONE;
  result->error_msg = "";
  result->operations_completed = operations_completed;
  result->operations_total = static_cast<uint16_t>(goal->operations.size());
  action_server_->succeeded_current(result);

  RCLCPP_INFO(
    get_logger(), "Operation goal completed: %d/%d operations",
    operations_completed, result->operations_total);
}

void
OperationsServer::pause_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "Pausing operations server");
  paused_ = true;

  for (auto & [action_name, plugin] : operations_) {
    plugin->pause();
  }

  response->success = true;
  response->message = "Operations server paused";
}

void
OperationsServer::resume_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "Resuming operations server");
  paused_ = false;

  for (auto & [action_name, plugin] : operations_) {
    plugin->resume();
  }

  response->success = true;
  response->message = "Operations server resumed";
}

rcl_interfaces::msg::SetParametersResult
OperationsServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (name.find('.') != std::string::npos) {
      continue;
    }

    if (!dynamic_params_lock_.try_lock()) {
      RCLCPP_WARN(
        get_logger(),
        "Unable to dynamically change Parameters while the server is currently running");
      result.successful = false;
      result.reason =
        "Unable to dynamically change Parameters while the server is currently running";
      return result;
    }

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "operation_frequency") {
        operation_frequency_ = parameter.as_double();
      }
    }

    dynamic_params_lock_.unlock();
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_operations_server

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(nav2_operations_server::OperationsServer)
