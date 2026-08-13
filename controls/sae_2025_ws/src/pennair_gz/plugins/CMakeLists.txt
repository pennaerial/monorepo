# add all plugins here
add_subdirectory(dynamic_detachable_joint)


# prepends the install directory of plugin .so files into GZ_SIM_SYSTEM_PLUGIN_PATH on sourcing
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/gz_plugin_path.sh.in"
)
