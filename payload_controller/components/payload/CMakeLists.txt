set(srcs
    payload.cpp
    ekf.cpp
)

set(requires DDS encoder imu log freertos motor util camera)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES ${requires}
)
