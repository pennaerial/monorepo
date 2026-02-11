cmake_minimum_required(VERSION 3.8)
project(custom_gz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS profiler)
find_package(gz-msgs10 REQUIRED)
find_package(custom_gz_msgs REQUIRED)

# DynamicDetachableJoint plugin
add_library(DynamicDetachableJoint SHARED
  src/dynamic_detachable_joint/DynamicDetachableJoint.cc
)

target_include_directories(DynamicDetachableJoint PRIVATE
  src/dynamic_detachable_joint
)

target_link_libraries(DynamicDetachableJoint
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-common5::gz-common5-profiler
  gz-msgs10::gz-msgs10
  custom_gz_msgs-msgs
)

install(TARGETS DynamicDetachableJoint
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
