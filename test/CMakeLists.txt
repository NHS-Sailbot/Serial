add_executable(serial_master_test master_test.cpp)
add_test(serial_master_test serial_master_test)

target_link_libraries(serial_master_test PRIVATE ${lib_name})
target_include_directories(serial_master_test PRIVATE ${lib_include_dir})

target_compile_options(serial_master_test PRIVATE -Werror)
target_compile_features(serial_master_test PRIVATE cxx_std_17)
