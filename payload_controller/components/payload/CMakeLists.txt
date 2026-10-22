set(srcs
    payload.cpp
    ekf.cpp
)

set(requires DDS encoder imu log freertos motor)

# encoder_impl.hpp/motor_impl.hpp expose the SITL driver headers on linux,
# which include sitl_runtime.hpp and gz headers.
if(IDF_TARGET STREQUAL "linux")
    list(APPEND requires sitl)
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES ${requires}
)

if(IDF_TARGET STREQUAL "linux")
    include("${CMAKE_SOURCE_DIR}/cmake/gz_deps.cmake")
    link_gz_transport(${COMPONENT_LIB})
endif()
