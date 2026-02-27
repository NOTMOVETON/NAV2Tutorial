cmake_minimum_required(VERSION 3.5)
project(nav2_operations_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OperationCommand.msg"
  "msg/OperationAtPose.msg"
  "action/ExecuteOperation.action"
  "action/FollowPathWithOperations.action"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
