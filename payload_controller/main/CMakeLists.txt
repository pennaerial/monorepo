
SET (priv_requires imu encoder DDS)

if(IDF_TARGET STREQUAL "linux")
    list(APPEND priv_requires "sitl")
endif()

idf_component_register(SRCS "payload_controller.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES ${priv_requires})
