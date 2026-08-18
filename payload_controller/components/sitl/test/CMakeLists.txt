include(${CMAKE_SOURCE_DIR}/cmake/tests.cmake)


register_linux_test(
    sitl_tests
    test_convert.cpp
)
