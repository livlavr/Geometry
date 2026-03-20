add_executable(geometry_tests
    tests.cpp
)

target_link_libraries(geometry_tests
    PRIVATE
    linalg-geometry
    gtest_main
)

include(GoogleTest)
gtest_discover_tests(geometry_tests)
