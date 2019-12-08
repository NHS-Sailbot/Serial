link_libraries(debug_lib)
include_directories(${serial_include_dir})
include_directories(${debug_include_dir})

if (WIN32)
    set(sources ${sources} plat/windows/windows_serial.cpp)
elseif (APPLE)
    set(sources ${sources} plat/macos/macos_serial.cpp)
elseif (UNIX)
    set(sources ${sources} plat/linux/linux_serial.cpp)
endif()

add_library(${PROJECT_NAME} ${sources})
