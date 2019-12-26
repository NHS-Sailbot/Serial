if (WIN32)
set(sources ${sources}
    plat/windows/serial.cpp
)
elseif (APPLE)
set(sources ${sources} 
    plat/macos/serial.cpp
)
elseif (UNIX)
set(sources ${sources} 
    plat/linux/serial.cpp
)
endif()

add_library(${lib_name} ${sources})

target_link_libraries(${lib_name} PRIVATE debug1.0.0)
target_include_directories(${lib_name} PRIVATE ${lib_include_dir})

target_compile_options(${lib_name} PRIVATE -Werror)
target_compile_features(${lib_name} PRIVATE cxx_std_17)

install(TARGETS ${lib_name} DESTINATION lib)
install(DIRECTORY ${lib_include_dir}/HENRY DESTINATION include)
