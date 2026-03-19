# Writing Custom Services in ROS2 Navigation2

- [Overview](#overview)
- [Requirements](#requirements)
- [Tutorial Steps](#tutorial-steps)
  - [1 — Define Custom Messages and Actions](#1--define-custom-messages-and-actions)
  - [2 — Implement the OperationsServer](#2--implement-the-operationsserver)
  - [3 — Implement BT Nodes](#3--implement-bt-nodes)
  - [4 — Create a Navigator Plugin](#4--create-a-navigator-plugin)
  - [5 — Configure Behavior Trees](#5--configure-behavior-trees)
  - [6 — Pass the Plugins Through the Params File](#6--pass-the-plugins-through-the-params-file)
  - [7 — Run the Demo](#7--run-the-demo)

---

## Overview

This tutorial shows how to integrate custom control into Nav2 by building a task server, connecting it to the behavior tree via custom BT nodes, and accepting goals for it through a custom navigator plugin — all as an overlay with no changes to Nav2 source code.

As a running example we use a lawnmower robot that needs to turn a blade on and off and rotate a servo camera at specific poses along a path. Nav2 handles planning and path following reliably, but provides no built-in mechanism for saying *"when the robot reaches this pose, turn the blade on."* The stack we build here adds that capability.

The full pipeline has three layers:

- An **OperationsServer** lifecycle node that handles blade and camera commands.
- **BT nodes** (`ExecuteOperation` and `OperationUpdater`) that invoke the server from inside a behavior tree.
- A **navigator plugin** (`NavigateWithOperations`) that accepts goals combining a path with pose-triggered operations and writes them to the BT blackboard.

---

## Requirements

- ROS 2 (binary or build-from-source)
- Nav2 (including dependencies)
- Docker (for the provided demo environment)

---

## Tutorial Steps

### 1 — Define Custom Messages and Actions

We need four interface files that define the data flowing through the pipeline.

**`OperationCommand.msg`** is the fundamental data unit. The `action_name` field routes to the correct handler inside the server; the typed arrays carry parameters without requiring a new message type for every actuator.

```
string    action_name
float32[] float_states
bool[]    bool_states
int32[]   int_states
string[]  string_states
```

**`OperationAtPose.msg`** pairs a trigger pose with a list of operations to fire there. Multiple operations at the same pose are executed sequentially.

```
geometry_msgs/PoseStamped pose
OperationCommand[]        operations
```

**`ExecuteOperation.action`** is the interface the `OperationsServer` exposes to the behavior tree.

```
OperationCommand[] operations
---
uint16 error_code
uint16 NONE=0
uint16 UNKNOWN_OPERATION=20001
uint16 OPERATION_FAILED=20002
uint16 CANCELLED=20003
---
string current_operation_name
uint16 operations_completed
uint16 operations_total
```

**`FollowPathWithOperations.action`** is the top-level interface accepted by the navigator plugin. The `behavior_tree` field follows Nav2 convention: if empty, the navigator falls back to its configured default BT file.

```
string                                  behavior_tree
nav_msgs/Path                           path
nav2_operations_msgs/OperationAtPose[]  operations_at_poses
---
std_msgs/Empty result
uint16         error_code
---
uint16 operations_completed
```

CMake and package metadata files can be found in `workspace/nav2_operations_msgs`.

---

### 2 — Implement the OperationsServer

The server is a standard Nav2 lifecycle node that wraps a `SimpleActionServer<ExecuteOperation>`. Its job is to receive operation goals, dispatch each command by `action_name`, and run them with cancellation support. Server itself is very similar to existing `Controller Server`, when response arrives main_loop is executed and while loop ticking time consuming operations with `OperationsServer.frequency` Hz.

The main execution loop iterates over each command in the goal:

```cpp
const uint16_t operations_total = goal->operations.size();

for (const auto & op : goal->operations) {
  // Publish feedback so the BT navigator can forward progress to the action client
  auto feedback = std::make_shared<ExecuteOperation::Feedback>();
  feedback->current_operation_name = op.action_name;
  feedback->operations_completed = operations_completed;
  feedback->operations_total = operations_total;
  action_server_->publish_feedback(feedback);

  if (op.action_name == "blade") {
    // Publish the blade state directly — no feedback loop needed
    auto msg = std_msgs::msg::Bool();
    msg.data = op.bool_states[0];
    blade_pub_->publish(msg);

  } else if (op.action_name == "camera") {
    // Camera rotation is time-driven: step at operation_frequency until done
    setCameraGoal(op);
    while (rclcpp::ok()) {
      if (action_server_->is_cancel_requested()) {
        result->error_code = Result::CANCELLED;
        action_server_->terminate_all(result);
        return;
      }
      stepCamera();
      action_server_->publish_feedback(feedback);
      if (cameraIsDone()) { break; }
      loop_rate.sleep();
    }

  } else {
    result->error_code = Result::UNKNOWN_OPERATION;
    action_server_->terminate_current(result);
    return;
  }
  operations_completed++;
}

result->operations_completed = operations_completed;
result->operations_total = operations_total;
action_server_->succeeded_current(result);
```

The `stepCamera` method computes the elapsed time since the previous call and advances `current_yaw_` by `max_rotation_speed * dt`, publishing the intermediate angle on each step. When `|current_yaw_ - target_yaw_|` falls below a tolerance, `cameraIsDone` returns true.

---

### 3 — Implement BT Nodes

Two BT nodes connect the behavior tree to the `OperationsServer`.

**`ExecuteOperationAction`** inherits from `nav2_behavior_tree::BtActionNode<ExecuteOperation>`. Using `BtActionNode` is important: it returns `RUNNING` on each tick while the action server is still executing. This means a long-running operation such as a camera sweep can coexist with `FollowPath` inside a `Parallel` control node, with neither blocking the other.

```cpp
void on_tick() override
{
  std::vector<OperationCommand> ops;
  getInput("operations", ops);
  goal_.operations = ops;
}
```

Operations are specified inline in BT XML as `"blade;bool;true"` or `"camera;float;90.0"`, with multiple operations separated by `|`. To support this, we register a `convertFromString` specialization that BehaviorTree.CPP calls automatically when populating the port:

```cpp
// Format: "action_name;type;value", multiple operations separated by '|'
// Examples:
//   "blade;bool;true"    -> OperationCommand{action_name="blade", bool_states=[true]}
//   "camera;float;180.0" -> OperationCommand{action_name="camera", float_states=[180.0]}

inline nav2_operations_msgs::msg::OperationCommand
parseOperationString(const std::string & str)
{
  nav2_operations_msgs::msg::OperationCommand cmd;
  // split "action_name;type;value" by ';'
  cmd.action_name = parts[0];
  if (type == "bool")   cmd.bool_states.push_back(value == "true");
  if (type == "float")  cmd.float_states.push_back(std::stof(value));
  if (type == "int")    cmd.int_states.push_back(std::stoi(value));
  if (type == "string") cmd.string_states.push_back(value);
  return cmd;
}

template<>
inline std::vector<nav2_operations_msgs::msg::OperationCommand>
convertFromString<std::vector<nav2_operations_msgs::msg::OperationCommand>>(StringView str)
{
  // split by '|', call parseOperationString for each part
  std::vector<nav2_operations_msgs::msg::OperationCommand> result;
  // ...
  return result;
}
```

**`OperationUpdater`** is a decorator node that monitors TF and fires pose-triggered operations as the robot moves. On each tick it runs through the following logic:

1. Spins pending callbacks to get fresh TF data.
2. Re-reads `operations_at_poses` from the blackboard. If the goal has changed since the last tick, clears all pending triggers and rebuilds the list from scratch, handling goal preemption cleanly.
3. Looks up the current robot pose via TF. Returns `FAILURE` immediately if TF is unavailable rather than operating on a stale pose.
4. If a multi-tick child operation is already in progress, ticks the child and propagates its status. Sets `performing_ = false` once the child finishes.
5. Otherwise scans the pending trigger list and fires the first one within `trigger_distance`, erasing it so each trigger fires at most once per goal.

When no trigger is nearby the decorator returns `SUCCESS`, signalling to a `ReactiveSequence` that it should proceed to tick `FollowPath` normally. When a trigger fires and the child returns `RUNNING`, the sequence halts `FollowPath` until the operation finishes.

```cpp
// if a multi-tick operation is already running, keep ticking its child
if (performing_) {
  RCLCPP_INFO(node_->get_logger(), "OperationUpdater: PERFORMING");
  const BT::NodeStatus child_state = child_node_->executeTick();
  if (child_state != BT::NodeStatus::RUNNING) {
    // Operation finished (SUCCESS or FAILURE) — clear the flag and propagate
    performing_ = false;
    RCLCPP_INFO(node_->get_logger(), "OperationUpdater: STOP PERFORMING");
    return child_state;
  }
  return BT::NodeStatus::RUNNING;
}

// scan pending triggers and fire the first one in range
for (auto & [idx, entry] : h_) {
  const auto & [trigger_pose, operations] = entry;
  double dx = current_pose_.pose.position.x - trigger_pose.pose.position.x;
  double dy = current_pose_.pose.position.y - trigger_pose.pose.position.y;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist < trigger_distance_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "OperationUpdater: triggered at index %d, distance %.2f, %zu operations",
      idx, dist, operations.size());

    // Write the operations list for the child ExecuteOperation node to read
    setOutput("current_operations", operations);
    // Erase so this trigger never fires again for the current goal
    h_.erase(idx);

    // Increment the counter the navigator plugin reads for action feedback
    uint16_t count = 0;
    [[maybe_unused]] auto res = config().blackboard->get("operations_completed", count);
    config().blackboard->set("operations_completed", static_cast<uint16_t>(count + 1));

    const BT::NodeStatus child_state = child_node_->executeTick();
    if (child_state == BT::NodeStatus::RUNNING) {
      // Child needs more ticks — mark as in-progress so Phase 3 handles it next time
      RCLCPP_INFO(node_->get_logger(), "OperationUpdater: PERFORMING");
      performing_ = true;
    }
    return child_state;
  }
}

// No trigger in range - signal the ReactiveSequence to proceed to FollowPath
return BT::NodeStatus::SUCCESS;
```

Exporting BT nodes and the navigator plugin via `pluginlib` is a standard ROS 2 operation and is not covered in detail here — refer to the [Nav2 navigator plugin tutorial](https://docs.nav2.org/plugin_tutorials/docs/writing_new_navigator_plugin.html) for the full pattern.

---

### 4 — Create a Navigator Plugin

The navigator plugin is the entry point for external clients. Standard Nav2 navigators handle `NavigateToPose` or `NavigateThroughPoses` — neither carries an `operations_at_poses` field. Our plugin accepts `FollowPathWithOperations` and writes its fields to the BT blackboard so the BT nodes can read them during execution.

The plugin inherits from `nav2_core::BehaviorTreeNavigator`. A detailed walkthrough of the base class API can be found in the [official Nav2 navigator plugin tutorial](https://docs.nav2.org/plugin_tutorials/docs/writing_new_navigator_plugin.html). We highlight only the methods specific to this integration.

**`configure`** initialises blackboard keys with sane defaults. BT nodes may read blackboard keys before the first goal arrives — for example when the tree is first loaded — so setting defaults here prevents uninitialized-read errors at runtime.

```cpp
bool NavigateWithOperations::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother>)
{
  // ... declare and read parameters ...

  auto bb = bt_action_server_->getBlackboard();
  bb->set<nav_msgs::msg::Path>(path_blackboard_id_, {});
  bb->set<std::vector<OperationAtPose>>(operations_at_poses_blackboard_id_, {});
  bb->set<uint16_t>(operations_completed_blackboard_id_, 0);
  return true;
}
```

**`goalReceived`** loads the requested BT file and writes the path and operation triggers to the blackboard so the running tree can pick them up immediately.

```cpp
bool NavigateWithOperations::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  if (!bt_action_server_->loadBehaviorTree(goal->behavior_tree)) {
    RCLCPP_ERROR(logger_, "BT file not found: %s. Navigation canceled.",
      goal->behavior_tree.c_str());
    return false;
  }
  auto bb = bt_action_server_->getBlackboard();
  bb->set<nav_msgs::msg::Path>(path_blackboard_id_, goal->path);
  bb->set<std::vector<OperationAtPose>>(
    operations_at_poses_blackboard_id_, goal->operations_at_poses);
  bb->set<uint16_t>(operations_completed_blackboard_id_, 0);
  return true;
}
```

**`onLoop`** reads the `operations_completed` counter that `OperationUpdater` maintains on the blackboard and forwards it to the action client as feedback.

**`goalCompleted`** clears all blackboard keys after each goal. Without this, a subsequent goal that reuses the same BT file would find stale `operations_at_poses` and fire outdated triggers.

```cpp
void NavigateWithOperations::goalCompleted(
  ActionT::Result::SharedPtr result,
  const nav2_behavior_tree::BtStatus final_bt_status)
{
  result->error_code = /* map final_bt_status to code */;

  auto bb = bt_action_server_->getBlackboard();
  bb->set<nav_msgs::msg::Path>(path_blackboard_id_, {});
  bb->set<std::vector<OperationAtPose>>(operations_at_poses_blackboard_id_, {});
  bb->set<uint16_t>(operations_completed_blackboard_id_, 0);
}
```

**`onPreempt`** accepts a new goal mid-navigation only if it requests the same BT file. Switching BT files would require hard cancellation of the current tree, so mismatched requests are rejected and the client is asked to cancel and resend.

```cpp
void NavigateWithOperations::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() || ...) {
    auto pending = bt_action_server_->acceptPendingGoal();
    auto bb = bt_action_server_->getBlackboard();
    bb->set<nav_msgs::msg::Path>(path_blackboard_id_, pending->path);
    bb->set<std::vector<OperationAtPose>>(
      operations_at_poses_blackboard_id_, pending->operations_at_poses);
    bb->set<uint16_t>(operations_completed_blackboard_id_, 0);
  } else {
    RCLCPP_WARN(logger_, "Preemption rejected: BT file mismatch. "
      "Cancel and resend to change BT.");
    bt_action_server_->terminatePendingGoal();
  }
}
```

---

### 5 — Configure Behavior Trees

We provide two behavior trees.

**`follow_path_with_operations.xml`** is the general-purpose tree. `OperationUpdater` monitors the robot's pose and fires operations as trigger poses are approached; `FollowPath` runs whenever no operation is active.

```xml
<ReactiveSequence>
  <OperationUpdater global_frame="map" robot_base_frame="base_link"
                    current_operations="{current_operations}">
    <ExecuteOperation operations="{current_operations}"/>
  </OperationUpdater>
  <TruncatePathLocal distance_forward="5.0" distance_backward="0.1"
                     input_path="{path}" output_path="{truncated_path}"/>
  <FollowPath path="{truncated_path}" controller_id="FollowPath"
              goal_checker_id="goal_checker"/>
</ReactiveSequence>
```

In a `ReactiveSequence`, a child returning `RUNNING` halts all following siblings. `OperationUpdater` returns `SUCCESS` when idle, so `FollowPath` ticks normally in that case. When a trigger fires, the decorator returns `RUNNING` and `FollowPath` is halted until the operation completes. To run both concurrently, replace `ReactiveSequence` with `Parallel`.

**`predefined_operations.xml`** is a sequential tree useful for fixed-route robots or startup/teardown sequences where operations are known at design time.

```xml
<Sequence>
  <DriveOnHeading dist_to_travel="0.2" speed="0.3" time_allowance="50"/>
  <ExecuteOperation operations="blade;bool;true"/>
  <DriveOnHeading dist_to_travel="3.0" speed="0.3" time_allowance="50"/>
  <ExecuteOperation operations="blade;bool;false|camera;float;0.0"/>
</Sequence>
```

The `|` separator sends multiple operations in a single action goal. The server executes them sequentially in order.

---

### 6 — Pass the Plugins Through the Params File

Three sections of the parameter file must stay consistent with each other.

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    navigators: ["navigate_with_operations"]
    navigate_with_operations:
      plugin: nav2_operations_navigator::NavigateWithOperations
    default_follow_path_with_operations_bt_xml:
      "/path/to/nav2_operations_demo/bt/follow_path_with_operations.xml"
    plugin_lib_names:
      - nav2_execute_operation_action_bt_node
      - nav2_operation_updater_decorator_bt_node

operations_server:
  ros__parameters:
    operation_frequency: 10.0
    camera:
      max_rotation_speed: 45.0

lifecycle_manager:
  ros__parameters:
    node_names:
      - controller_server
      - bt_navigator
      - operations_server
```

The `plugin_lib_names` list tells the BT Navigator which shared libraries to load at startup. If a BT XML references a node type not in this list, tree loading will fail with a `node type not registered` error.

---

### 7 — Run the Demo

The `docker/` directory in the repository contains a self-contained build and run environment.

```bash
cd docker
./run.sh --build   # first time only
./run.sh           # start the container
```

Inside the container:

```bash
colcon build && source install/setup.bash
ros2 launch nav2_operations_demo operations_launch.py
```

To test the sequential tree without needing RViz:

```bash
ros2 action send_goal /navigate_with_operations \
  nav2_operations_msgs/action/FollowPathWithOperations \
  "{behavior_tree: '/ros2_ws/src/nav2_operations_demo/bt/predefined_operations.xml',
    path: {header: {frame_id: map}, poses: []},
    operations_at_poses: []}"
```

To test the reactive tree, open RViz2, load the config at `/ros2_ws/src/config.rviz`, and use the 2D Goal Pose button. The demo node will plan a path and distribute blade and camera triggers along it automatically.

[Video demonstration](https://www.youtube.com/watch?v=r7YYTKF0jlY)
