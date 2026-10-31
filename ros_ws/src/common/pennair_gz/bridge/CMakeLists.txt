# --- Generated parameters ---
generate_parameter_library(
    pennair_gz_bridge_parameters
    config/pennair_gz_bridge_params.schema.yaml
)

# --- Plugin shared library ---
# All bridge plugins are compiled into this single shared library.
# To add a new bridge: add its .cpp here and register it in plugins.xml.
add_library(pennair_gz_bridge_plugins SHARED
  attach_detach_bridge.cpp
)

target_include_directories(pennair_gz_bridge_plugins PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

ament_target_dependencies(pennair_gz_bridge_plugins
  rclcpp
  sim_interfaces
  pluginlib
)

target_link_libraries(pennair_gz_bridge_plugins
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  pennair_gz_msgs-msgs
  pennair_gz_bridge_parameters
)

# Register plugins with pluginlib
# pluginlib_export_plugin_description_file(rclcpp plugins.xml)

##############################################################

# --- Loader executable ---
# Generic executable that loads a bridge plugin by class name.
add_executable(bridge_loader
  bridge_loader.cpp
)

ament_target_dependencies(bridge_loader
  rclcpp
  pluginlib
)

# --- Install ---
install(TARGETS pennair_gz_bridge_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS bridge_loader
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY include/
#   DESTINATION include
# )

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
