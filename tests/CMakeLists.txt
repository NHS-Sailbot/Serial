include(CTest)

link_libraries(${PROJECT_NAME})
include_directories(${serial_include_dir})
include_directories(${debug_include_dir})

add_executable(serial_master_test master_test.cpp)
add_test(serial_master_test serial_master_test)
