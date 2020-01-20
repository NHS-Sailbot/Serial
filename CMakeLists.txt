cmake_minimum_required(VERSION 3.12)

if (not TARGET Serial)
    project(Serial VERSION 1.0.0)
    add_subdirectory(src)
endif()
