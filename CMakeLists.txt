cmake_minimum_required(VERSION 3.0.0)
project(serial VERSION 0.1.0)

set(project_include_dir "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_subdirectory(src)
add_subdirectory(tests)
