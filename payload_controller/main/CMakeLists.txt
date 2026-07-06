
SET (priv_requires imu encoder)

if(IDF_TARGET STREQUAL "linux")
    list(APPEND PRIV_REQUIRES_COMPONENTS "sitl")
endif()

idf_component_register(SRCS "payload_controller.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES ${priv_requires})
