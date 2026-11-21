cmake_minimum_required(VERSION 3.8)
project(path_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(payload_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# --- Path follower executable ---
add_executable(path_follower_node
  src/path_follower.cpp
  src/path_follower_main.cpp
)

ament_target_dependencies(path_follower_node
  rclcpp
  nav_msgs
  payload_interfaces
  tf2
  tf2_ros
)

install(TARGETS path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include, launch, config directories
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
