#ifndef NAV2_OPERATIONS_BT_NODES__BT_CONVERSIONS_HPP_
#define NAV2_OPERATIONS_BT_NODES__BT_CONVERSIONS_HPP_

#include <string>
#include <vector>
#include <sstream>

#include "behaviortree_cpp/behavior_tree.h"
#include "nav2_operations_msgs/msg/operation_command.hpp"

namespace BT
{

// Parse a single "action_name;type;value" string into OperationCommand
inline nav2_operations_msgs::msg::OperationCommand parseOperationString(const std::string & str)
{
  nav2_operations_msgs::msg::OperationCommand cmd;

  // Split by semicolons
  std::istringstream ss(str);
  std::string token;
  std::vector<std::string> parts;
  while (std::getline(ss, token, ';')) {
    parts.push_back(token);
  }

  if (parts.size() < 3) {
    throw RuntimeError("Invalid operation string format. Expected 'action_name;type;value', got: " + str);
  }

  cmd.action_name = parts[0];
  const std::string & type = parts[1];
  const std::string & value = parts[2];

  if (type == "bool") {
    cmd.bool_states.push_back(value == "true" || value == "1");
  } else if (type == "float") {
    cmd.float_states.push_back(std::stof(value));
  } else if (type == "int") {
    cmd.int_states.push_back(std::stoi(value));
  } else if (type == "string") {
    cmd.string_states.push_back(value);
  } else {
    throw RuntimeError("Unknown operation type: " + type);
  }

  return cmd;
}

template<>
inline std::vector<nav2_operations_msgs::msg::OperationCommand>
convertFromString<std::vector<nav2_operations_msgs::msg::OperationCommand>>(StringView str)
{
  std::vector<nav2_operations_msgs::msg::OperationCommand> result;

  // Multiple operations separated by '|'
  std::string s(str.data(), str.size());
  std::istringstream ss(s);
  std::string op_str;
  while (std::getline(ss, op_str, '|')) {
    if (!op_str.empty()) {
      result.push_back(parseOperationString(op_str));
    }
  }

  return result;
}

}  // namespace BT

#endif  // NAV2_OPERATIONS_BT_NODES__BT_CONVERSIONS_HPP_
