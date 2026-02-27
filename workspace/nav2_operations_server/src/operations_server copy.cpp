#include "nav2_operations_server/operations_server.hpp"

#include <chrono>
#include <functional>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace nav2_operations_server
{

OperationsServer::OperationsServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("operations_server", "", options),
  plugin_loader_("nav2_operations_server", "nav2_operations_server::Operation"),
  paused_(false)
{
  RCLCPP_INFO(get_logger(), "Creating OperationsServer");
}

OperationsServer::~OperationsServer()
{
  RCLCPP_INFO(get_logger(), "Destroying OperationsServer");
}

OperationsServer::CallbackReturn
OperationsServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring OperationsServer");

  // Read parameters
  declare_parameter("operation_frequency", 10.0);
  get_parameter("operation_frequency", operation_frequency_);

  declare_parameter("operation_plugins", std::vector<std::string>());
  get_parameter("operation_plugins", plugin_names_);

  // Load each plugin
  for (const auto & plugin_name : plugin_names_) {
    declare_parameter(plugin_name + ".plugin", std::string());
    declare_parameter(plugin_name + ".action_name", std::string());
    declare_parameter(plugin_name + ".instant", false);

    std::string plugin_type;
    std::string action_name;
    bool instant = false;

    get_parameter(plugin_name + ".plugin", plugin_type);
    get_parameter(plugin_name + ".action_name", action_name);
    get_parameter(plugin_name + ".instant", instant);

    RCLCPP_INFO(
      get_logger(), "Loading plugin '%s' of type '%s' for action '%s' (instant: %s)",
      plugin_name.c_str(), plugin_type.c_str(), action_name.c_str(),
      instant ? "true" : "false");

    try {
      auto plugin = plugin_loader_.createSharedInstance(plugin_type);
      plugin->configure(weak_from_this(), plugin_name);
      operations_[action_name] = plugin;
      instant_flags_[action_name] = instant;
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to load plugin '%s': %s",
        plugin_name.c_str(), ex.what());
      return CallbackReturn::FAILURE;
    }
  }

  // Create pause/resume services
  pause_service_ = create_service<std_srvs::srv::Trigger>(
    "operations_server/pause",
    std::bind(&OperationsServer::pause_callback, this, _1, _2));

  resume_service_ = create_service<std_srvs::srv::Trigger>(
    "operations_server/resume",
    std::bind(&OperationsServer::resume_callback, this, _1, _2));

  // Create action server
  action_server_ = rclcpp_action::create_server<ExecuteOperation>(
    this,
    "execute_operation",
    std::bind(&OperationsServer::handle_goal, this, _1, _2),
    std::bind(&OperationsServer::handle_cancel, this, _1),
    std::bind(&OperationsServer::handle_accepted, this, _1));

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

  // Register bond with lifecycle manager so it knows this node is alive
  createBond();

  RCLCPP_INFO(get_logger(), "OperationsServer activated");
  return CallbackReturn::SUCCESS;
}

OperationsServer::CallbackReturn
OperationsServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating OperationsServer");

  // Destroy bond with lifecycle manager
  destroyBond();

  for (auto & [action_name, plugin] : operations_) {
    plugin->deactivate();
  }

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
  plugin_names_.clear();

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

rclcpp_action::GoalResponse
OperationsServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const ExecuteOperation::Goal> /*goal*/)
{
  RCLCPP_INFO(get_logger(), "Received operation goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
OperationsServer::handle_cancel(
  const std::shared_ptr<GoalHandleExecuteOperation> /*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel operation goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
OperationsServer::handle_accepted(
  const std::shared_ptr<GoalHandleExecuteOperation> goal_handle)
{
  std::thread{std::bind(&OperationsServer::execute, this, goal_handle)}.detach();
}

void
OperationsServer::execute(
  const std::shared_ptr<GoalHandleExecuteOperation> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing operation goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ExecuteOperation::Result>();
  auto feedback = std::make_shared<ExecuteOperation::Feedback>();

  result->operations_total = static_cast<uint16_t>(goal->operations.size());
  feedback->operations_total = static_cast<uint16_t>(goal->operations.size());
  uint16_t operations_completed = 0;

  rclcpp::Rate rate(operation_frequency_);

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
      goal_handle->abort(result);
      return;
    }

    // Wait while paused before starting the operation
    while (paused_ && rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->error_code = ExecuteOperation::Result::CANCELLED;
        result->error_msg = "Cancelled while paused";
        result->operations_completed = operations_completed;
        goal_handle->canceled(result);
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
    goal_handle->publish_feedback(feedback);

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
        // Check for cancellation
        if (goal_handle->is_canceling()) {
          RCLCPP_INFO(get_logger(), "Operation '%s' cancelled", action_name.c_str());
          plugin->stop();
          result->error_code = ExecuteOperation::Result::CANCELLED;
          result->error_msg = "Operation cancelled";
          result->operations_completed = operations_completed;
          goal_handle->canceled(result);
          return;
        }

        // Handle pause
        if (paused_) {
          plugin->pause();
          while (paused_ && rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
              plugin->stop();
              result->error_code = ExecuteOperation::Result::CANCELLED;
              result->error_msg = "Cancelled while paused";
              result->operations_completed = operations_completed;
              goal_handle->canceled(result);
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

        rate.sleep();
      }
      operations_completed++;
    }

  }

  // Set final result
  result->error_code = ExecuteOperation::Result::NONE;
  result->error_msg = "";
  result->operations_completed = operations_completed;
  result->operations_total = static_cast<uint16_t>(goal->operations.size());
  goal_handle->succeed(result);

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

}  // namespace nav2_operations_server
