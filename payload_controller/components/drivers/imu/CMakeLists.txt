set(srcs imu.cpp)
set(priv_requires DDS)
set(requires util)

set(sitl_srcs
    imu_sitl.cpp
)

set(hardware_srcs
    imu_hw.cpp
    lsm6dso.cpp
    imu.cpp
)


if(IDF_TARGET STREQUAL linux)
    list(APPEND srcs ${sitl_srcs})
    list(APPEND priv_requires sitl)
else()
    list(APPEND srcs ${hardware_srcs})
    list(APPEND priv_requires esp_driver_gpio esp_driver_i2c esp_timer)
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
