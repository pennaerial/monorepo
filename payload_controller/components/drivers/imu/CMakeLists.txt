set(srcs imu.cpp)
set(priv_requires DDS)
set(requires util)

# TODO: Remove these cmake defined args for kconfig later
set(IMU_MODEL "hw" CACHE STRING "Hardware IMU model")
set_property(CACHE IMU_MODEL PROPERTY STRINGS hw lsm6dso)

if(IDF_TARGET STREQUAL "linux")
    list(APPEND srcs "imu_sitl.cpp")
elseif(IMU_MODEL STREQUAL "hw")
    list(APPEND srcs "imu_hw.cpp")
elseif(IMU_MODEL STREQUAL "lsm6dso")
    list(APPEND srcs "lsm6dso.cpp")
else()
    message(FATAL_ERROR "Unsupported IMU_MODEL: ${IMU_MODEL}")
endif()


if(IDF_TARGET STREQUAL linux)
    list(APPEND priv_requires sitl)
else()
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
