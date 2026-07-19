
SET (priv_requires imu encoder pc_xrce_dds_client)

if(IDF_TARGET STREQUAL "linux")
    list(APPEND priv_requires "sitl")
endif()

idf_component_register(SRCS "payload_controller.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES ${priv_requires})
