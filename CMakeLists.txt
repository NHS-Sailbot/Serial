cmake_minimum_required(VERSION 3.12)

project(Serial VERSION 1.0.0)

if (NOT TARGET Serial)
	set(Headers
        inc/Henry/Serial.hpp
    )
    set(Sources
    )

    if (WIN32)
        set(Sources ${Sources}
            src/plat/WindowsSerial.cpp
        )
    elseif (APPLE)
        set(Sources ${Sources})
    elseif (UNIX)
        set(Sources ${Sources}
            src/plat/LinuxSerial.cpp
        )
    endif()

    add_library(Serial ${Sources} ${Headers})

    target_compile_features(Serial PUBLIC cxx_std_17)
    target_include_directories(Serial PUBLIC ${CMAKE_CURRENT_LIST_DIR}/inc)
endif()
