cmake_minimum_required(VERSION 3.5)
project(nav2_operations_bt_nodes)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_operations_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp
  nav2_behavior_tree
  nav2_operations_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
)

# Each BT node as separate shared library
add_library(nav2_execute_operation_action_bt_node SHARED
  src/action/execute_operation_action.cpp)
target_include_directories(nav2_execute_operation_action_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nav2_execute_operation_action_bt_node ${dependencies})

add_library(nav2_cancel_operation_action_bt_node SHARED
  src/action/cancel_operation_action.cpp)
target_include_directories(nav2_cancel_operation_action_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nav2_cancel_operation_action_bt_node ${dependencies})

add_library(nav2_pause_operations_server_bt_node SHARED
  src/action/pause_operations_server.cpp)
target_include_directories(nav2_pause_operations_server_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nav2_pause_operations_server_bt_node ${dependencies})

add_library(nav2_resume_operations_server_bt_node SHARED
  src/action/resume_operations_server.cpp)
target_include_directories(nav2_resume_operations_server_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nav2_resume_operations_server_bt_node ${dependencies})

add_library(nav2_operation_updater_decorator_bt_node SHARED
  src/decorator/operation_updater_node.cpp)
target_include_directories(nav2_operation_updater_decorator_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nav2_operation_updater_decorator_bt_node ${dependencies})

# List of all BT node libraries
set(bt_node_libs
  nav2_execute_operation_action_bt_node
  nav2_cancel_operation_action_bt_node
  nav2_pause_operations_server_bt_node
  nav2_resume_operations_server_bt_node
  nav2_operation_updater_decorator_bt_node
)

# Required so BTCPP_EXPORT expands to extern "C" __attribute__((visibility("default")))
# instead of "static", making BT_RegisterNodesFromPlugin visible in the shared library
foreach(bt_plugin ${bt_node_libs})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${bt_node_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES nav2_tree_nodes.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${bt_node_libs})
ament_export_dependencies(${dependencies})

ament_package()
