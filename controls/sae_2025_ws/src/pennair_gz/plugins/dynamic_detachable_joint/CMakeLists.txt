add_library(DynamicDetachableJoint SHARED
  DynamicDetachableJoint.cpp
)

# private header, so allow use of local include with no namespaces
target_include_directories(DynamicDetachableJoint PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(DynamicDetachableJoint
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-common5::gz-common5-profiler
  gz-msgs10::gz-msgs10
  custom_gz_msgs-msgs
)

# goes to install/pennair_gz/lib/libDynamicDetachableJoint.so
install(TARGETS DynamicDetachableJoint
  LIBRARY DESTINATION lib
)

