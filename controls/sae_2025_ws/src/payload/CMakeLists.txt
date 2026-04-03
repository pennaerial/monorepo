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
find_package(generate_parameter_library REQUIRED)

option(BUILD_SIM "Build simulation controller (OFF builds GPIO controller instead)" ON)
if(BUILD_SIM)
  find_package(gz-transport13 REQUIRED)
  find_package(gz-msgs10 REQUIRED)
else()
  find_library(LGPIO_LIBRARY NAMES lgpio REQUIRED)
  find_path(LGPIO_INCLUDE_DIR NAMES lgpio.h REQUIRED)
endif()

include_directories(include)

#Generate Parameter Library
generate_parameter_library(
  payload_parameters
  config/payload_params.schema.yaml
)

# --- Controller plugin shared library ---
add_library(payload_controllers SHARED
  $<IF:$<BOOL:${BUILD_SIM}>,src/sim_controller.cpp,src/gpio_controller.cpp>
)

ament_target_dependencies(payload_controllers
  rclcpp
  payload_interfaces
  pluginlib
)

target_link_libraries(payload_controllers
  payload_parameters
)

if(BUILD_SIM)
  target_link_libraries(payload_controllers
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
  )
  target_compile_definitions(payload_controllers PRIVATE BUILD_SIM)
else()
  target_sources(payload_controllers PRIVATE
    src/encoder.cpp
    src/control_math.cpp
    src/motor.cpp
    src/servo.cpp
    src/gpio.cpp
  )
  target_link_libraries(payload_controllers ${LGPIO_LIBRARY})
  target_include_directories(payload_controllers PUBLIC ${LGPIO_INCLUDE_DIR})
  target_compile_definitions(payload_controllers PRIVATE BUILD_GPIO)

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

  # --- Servo test executable (hardware only) ---
  add_executable(test_servo
    test/test_servo.cpp
    src/servo.cpp
    src/gpio.cpp
  )
  target_include_directories(test_servo PRIVATE include ${LGPIO_INCLUDE_DIR})
  ament_target_dependencies(test_servo rclcpp)
  target_link_libraries(test_servo ${LGPIO_LIBRARY})
  install(TARGETS test_servo DESTINATION lib/${PROJECT_NAME})
endif()

pluginlib_export_plugin_description_file(payload plugins.xml)

# --- Payload executable ---
add_executable(payload
  src/payload.cpp
  src/main.cpp
)

ament_target_dependencies(payload
  rclcpp
  payload_interfaces
  pluginlib
)

target_link_libraries(payload
  payload_parameters
)

install(TARGETS payload_controllers
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

# --- Python payload agent script ---
install(PROGRAMS scripts/payload_agent.py DESTINATION lib/${PROJECT_NAME} RENAME payload_agent)





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
