set(srcs
    payload.cpp
    ekf.cpp
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES DDS encoder imu log freertos
)
