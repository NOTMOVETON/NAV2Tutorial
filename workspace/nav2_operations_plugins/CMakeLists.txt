cmake_minimum_required(VERSION 3.5)
project(nav2_operations_plugins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_operations_msgs REQUIRED)
find_package(nav2_operations_server REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_library(${PROJECT_NAME} SHARED
  src/control_rotor.cpp
  src/set_camera_yaw.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  pluginlib
  std_msgs
  nav2_operations_msgs
  nav2_operations_server
)

pluginlib_export_plugin_description_file(nav2_operations_server plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  pluginlib
  std_msgs
  nav2_operations_msgs
  nav2_operations_server
)

ament_package()
