set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.8)
project(vision_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(vision_manager src/vision_manager.cpp)
ament_target_dependencies(vision_manager rclcpp std_msgs)

install(TARGETS
  vision_manager
  DESTINATION lib/${PROJECT_NAME})


ament_package()
