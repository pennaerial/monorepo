set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.8)
project(pennair_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

# VisionManager exe
add_executable(vision_manager
    src/main.cpp
    src/vision_manager.cpp
)

ament_target_dependencies(vision_manager
    rclcpp
    std_msgs
    sensor_msgs
    pluginlib
)

# Dynamically loaded plugins
add_library(vision_plugins SHARED
    src/plugins/basic_vision.cpp
)


target_include_directories(vision_plugins PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(vision_plugins
    rclcpp
    pluginlib
    sensor_msgs
)

pluginlib_export_plugin_description_file(pennair_vision plugins.xml)




install(TARGETS
    vision_manager
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS vision_plugins
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

# Make headers available to other packages
install(
  DIRECTORY include/
  DESTINATION include
)


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs sensor_msgs pluginlib)

ament_package()
