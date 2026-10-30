idf_component_register(
    SRCS "pid.cpp" "feedforward.cpp" "pwmcontroller.cpp"
    INCLUDE_DIRS "."
    REQUIRES freertos
)
