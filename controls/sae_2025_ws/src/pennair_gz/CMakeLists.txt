cmake_minimum_required(VERSION 3.8)
project(pennair_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sae_gz_cmake REQUIRED)

sae_require_gazebo_harmonic()

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS profiler)
find_package(gz-msgs10 REQUIRED)
find_package(custom_gz_msgs REQUIRED)
find_package(sim_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)

add_subdirectory(plugins)
add_subdirectory(bridge)
# This must be on package level and doesn't work in a subdirectory
pluginlib_export_plugin_description_file(rclcpp bridge/plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
