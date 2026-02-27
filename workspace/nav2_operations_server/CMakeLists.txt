cmake_minimum_required(VERSION 3.5)
project(nav2_operations_server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_operations_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# Library with the server (so plugins can link against the Operation base class)
add_library(${PROJECT_NAME}_core SHARED
  src/operations_server.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rclcpp_action
  nav2_util
  pluginlib
  std_srvs
  nav2_operations_msgs
)

# Executable
add_executable(operations_server_node src/main.cpp)
target_link_libraries(operations_server_node ${PROJECT_NAME}_core)

# Install
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_core
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS operations_server_node DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rclcpp_action
  nav2_util
  pluginlib
  std_srvs
  nav2_operations_msgs
)
ament_export_targets(${PROJECT_NAME}Targets)

ament_package()
