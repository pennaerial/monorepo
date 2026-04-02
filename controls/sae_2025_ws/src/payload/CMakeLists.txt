cmake_minimum_required(VERSION 3.8)
project(payload)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(payload_interfaces REQUIRED)
find_package(pluginlib REQUIRED)

set(PAYLOAD_BUILD_SIM_CONTROLLER "AUTO" CACHE STRING
  "Build the SimController backend (AUTO, ON, OFF)")
set_property(CACHE PAYLOAD_BUILD_SIM_CONTROLLER PROPERTY STRINGS AUTO ON OFF)

set(PAYLOAD_BUILD_GPIO_CONTROLLER "AUTO" CACHE STRING
  "Build the GPIOController backend (AUTO, ON, OFF)")
set_property(CACHE PAYLOAD_BUILD_GPIO_CONTROLLER PROPERTY STRINGS AUTO ON OFF)

string(TOUPPER "${PAYLOAD_BUILD_SIM_CONTROLLER}" PAYLOAD_BUILD_SIM_CONTROLLER)
string(TOUPPER "${PAYLOAD_BUILD_GPIO_CONTROLLER}" PAYLOAD_BUILD_GPIO_CONTROLLER)

include_directories(include)

set(payload_controller_targets "")
set(build_payload_sim_controller FALSE)
set(build_payload_gpio_controller FALSE)

if(PAYLOAD_BUILD_SIM_CONTROLLER STREQUAL "ON")
  find_package(gz-transport13 REQUIRED)
  find_package(gz-msgs10 REQUIRED)
  set(build_payload_sim_controller TRUE)
elseif(PAYLOAD_BUILD_SIM_CONTROLLER STREQUAL "AUTO")
  find_package(gz-transport13 QUIET)
  find_package(gz-msgs10 QUIET)
  if(TARGET gz-transport13::gz-transport13 AND TARGET gz-msgs10::gz-msgs10)
    set(build_payload_sim_controller TRUE)
  else()
    message(STATUS
      "Skipping SimController because Gazebo development packages were not found.")
  endif()
elseif(NOT PAYLOAD_BUILD_SIM_CONTROLLER STREQUAL "OFF")
  message(FATAL_ERROR
    "PAYLOAD_BUILD_SIM_CONTROLLER must be one of: AUTO, ON, OFF")
endif()

if(build_payload_sim_controller)
  add_library(payload_sim_controller SHARED
    src/payload_parameters.cpp
    src/sim_controller.cpp
  )
  ament_target_dependencies(payload_sim_controller
    rclcpp
    payload_interfaces
    pluginlib
  )
  target_link_libraries(payload_sim_controller
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
  )
  target_compile_definitions(payload_sim_controller PRIVATE BUILD_SIM)
  list(APPEND payload_controller_targets payload_sim_controller)
endif()

if(PAYLOAD_BUILD_GPIO_CONTROLLER STREQUAL "ON")
  find_library(LGPIO_LIBRARY NAMES lgpio REQUIRED)
  find_path(LGPIO_INCLUDE_DIR NAMES lgpio.h REQUIRED)
  set(build_payload_gpio_controller TRUE)
elseif(PAYLOAD_BUILD_GPIO_CONTROLLER STREQUAL "AUTO")
  find_library(LGPIO_LIBRARY NAMES lgpio)
  find_path(LGPIO_INCLUDE_DIR NAMES lgpio.h)
  if(LGPIO_LIBRARY AND LGPIO_INCLUDE_DIR)
    set(build_payload_gpio_controller TRUE)
  else()
    message(STATUS
      "Skipping GPIOController because lgpio development files were not found. "
      "Install liblgpio-dev to enable the hardware backend.")
  endif()
elseif(NOT PAYLOAD_BUILD_GPIO_CONTROLLER STREQUAL "OFF")
  message(FATAL_ERROR
    "PAYLOAD_BUILD_GPIO_CONTROLLER must be one of: AUTO, ON, OFF")
endif()

if(build_payload_gpio_controller)
  add_library(payload_gpio_controller SHARED
    src/payload_parameters.cpp
    src/gpio_controller.cpp
    src/encoder.cpp
    src/control_math.cpp
  )
  ament_target_dependencies(payload_gpio_controller
    rclcpp
    payload_interfaces
    pluginlib
  )
  target_link_libraries(payload_gpio_controller ${LGPIO_LIBRARY})
  target_include_directories(payload_gpio_controller PUBLIC ${LGPIO_INCLUDE_DIR})
  target_compile_definitions(payload_gpio_controller PRIVATE BUILD_GPIO)
  list(APPEND payload_controller_targets payload_gpio_controller)

  # --- Motor test executable (hardware only) ---
  add_executable(test_motor
    test/test_motor.cpp
    src/motor.cpp
    src/servo.cpp
    src/gpio.cpp
    src/encoder.cpp
  )
  target_include_directories(test_motor PRIVATE include ${LGPIO_INCLUDE_DIR})
  ament_target_dependencies(test_motor rclcpp)
  target_link_libraries(test_motor ${LGPIO_LIBRARY})
  install(TARGETS test_motor DESTINATION lib/${PROJECT_NAME})
endif()

if(NOT payload_controller_targets)
  message(FATAL_ERROR
    "payload requires at least one controller backend. Enable the simulation "
    "backend by installing Gazebo development packages, or enable the hardware "
    "backend by installing liblgpio-dev.")
endif()

pluginlib_export_plugin_description_file(payload plugins.xml)

# --- Payload executable ---
add_executable(payload
  src/payload_parameters.cpp
  src/payload.cpp
  src/main.cpp
)

ament_target_dependencies(payload
  rclcpp
  payload_interfaces
  pluginlib
)

install(TARGETS ${payload_controller_targets}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS payload
  DESTINATION lib/${PROJECT_NAME}
)

# Install include, launch, config directories
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)





if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_control_math
    test/test_control_math.cpp
    src/control_math.cpp
  )
  if(TARGET test_control_math)
    target_include_directories(test_control_math PRIVATE include)
  endif()
endif()

ament_package()
