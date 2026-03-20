cmake_minimum_required(VERSION 3.20)
project(LinalgGeometry)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(linalg-geometry INTERFACE)

target_include_directories(linalg-geometry INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

enable_testing()
add_subdirectory(tests)
