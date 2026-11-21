set(srcs motor.cpp)
set(priv_requires "")
set(requires util)

if(IDF_TARGET STREQUAL "linux")
    list(APPEND srcs "motor_sitl.cpp")
    list(APPEND priv_requires sitl)
else()
    list(APPEND srcs "DRV8833.cpp")
    list(APPEND priv_requires esp_driver_ledc esp_driver_gpio)
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    PRIV_REQUIRES ${priv_requires}
    REQUIRES ${requires}
)

if(IDF_TARGET STREQUAL "linux")
    include("${CMAKE_SOURCE_DIR}/cmake/gz_deps.cmake")
    link_gz_transport(${COMPONENT_LIB})
endif()
