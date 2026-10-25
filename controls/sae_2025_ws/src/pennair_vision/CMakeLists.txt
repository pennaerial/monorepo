set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.19)
project(pennair_vision)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_components REQUIRED) # for vision_manager_component
find_package(ament_cmake_python REQUIRED) # for pennair_vision python module
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED) # for pennair_vision lazy publisher

include_directories(include)

add_library(vision_manager_component SHARED
    src/vision_manager.cpp
)

ament_target_dependencies(vision_manager_component
    rclcpp
    rosidl_runtime_cpp
    rclcpp_components
    std_msgs
    sensor_msgs
    pluginlib
    image_transport
    cv_bridge
)

rclcpp_components_register_nodes(vision_manager_component
    "pennair_vision::VisionManager"
)

add_executable(vision_manager
    src/main.cpp
    src/vision_manager.cpp
)

ament_target_dependencies(vision_manager
    rclcpp
    rosidl_runtime_cpp
    rclcpp_components
    std_msgs
    sensor_msgs
    pluginlib
    image_transport
    cv_bridge
)

# Dynamically loaded plugins
add_library(vision_plugins SHARED
    src/plugins/basic_vision.cpp
)

add_library(lazy_camera_publisher SHARED
    src/lazy_camera_publisher.cpp
)

target_include_directories(vision_plugins PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(lazy_camera_publisher PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(vision_plugins
    rclcpp
    rosidl_runtime_cpp
    pluginlib
    sensor_msgs
    image_transport
    cv_bridge
)

target_compile_features(vision_plugins PUBLIC cxx_std_20)
target_compile_features(lazy_camera_publisher PUBLIC cxx_std_20)

pluginlib_export_plugin_description_file(pennair_vision plugins.xml)

ament_target_dependencies(lazy_camera_publisher
    rclcpp
    rosidl_runtime_cpp
    sensor_msgs
    image_transport
    cv_bridge
)

install(TARGETS
    vision_manager_component
    vision_manager
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(PROGRAMS
    $<TARGET_FILE:vision_manager>
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

############## PENNAIR_VISION AMENT_PYTHON PACKAGE #############
ament_python_install_package(${PROJECT_NAME})

include(cmake/install_python_entry_point.cmake)
install_python_entry_point(basic_vision_py pennair_vision.basic_vision main)


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rosidl_runtime_cpp rclcpp std_msgs sensor_msgs pluginlib image_transport cv_bridge)

ament_package()
