cmake_minimum_required(VERSION 3.8)
project(pennair_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# lets clangd / VS Code find headers (build/compile_commands.json)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPath.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

# The service node lives in the same package as the interfaces it uses,
# so link against the generated typesupport target.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(planner_service SHARED src/planner_service.cpp)
target_include_directories(planner_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(planner_service "${cpp_typesupport_target}" Eigen3::Eigen)
ament_target_dependencies(planner_service
  geometry_msgs nav_msgs rclcpp rclcpp_components)

# Registers the component and also builds a standalone executable named planner_service_node
rclcpp_components_register_node(planner_service
  PLUGIN "pennair_planner::PlannerService"
  EXECUTABLE planner_service_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS planner_service
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
