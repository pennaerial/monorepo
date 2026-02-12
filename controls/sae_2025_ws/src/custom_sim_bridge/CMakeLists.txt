cmake_minimum_required(VERSION 3.8)
project(custom_sim_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sim_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(custom_gz_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

include_directories(include)

# --- Generated parameters ---
generate_parameter_library(
  custom_sim_bridge_parameters
  config/custom_sim_bridge_params.schema.yaml
)

# --- Plugin shared library ---
# All bridge plugins are compiled into this single shared library.
# To add a new bridge: add its .cpp here and register it in plugins.xml.
add_library(custom_sim_bridge_plugins SHARED
  src/attach_detach_bridge.cpp
)

ament_target_dependencies(custom_sim_bridge_plugins
  rclcpp
  sim_interfaces
  pluginlib
)

target_link_libraries(custom_sim_bridge_plugins
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  custom_gz_msgs-msgs
  custom_sim_bridge_parameters
)

# Register plugins with pluginlib
pluginlib_export_plugin_description_file(rclcpp plugins.xml)

# --- Loader executable ---
# Generic executable that loads a bridge plugin by class name.
# Usage: ros2 run custom_sim_bridge bridge_loader --ros-args -p bridge_type:=custom_sim_bridge::AttachDetachBridge
add_executable(bridge_loader
  src/bridge_loader.cpp
)

ament_target_dependencies(bridge_loader
  rclcpp
  pluginlib
)

# --- Install ---
install(TARGETS custom_sim_bridge_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS bridge_loader
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
