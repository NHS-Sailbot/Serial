set(Headers
    Serial.hpp
)
set(Sources
)

if (WIN32)
    set(Sources ${Sources})
elseif (APPLE)
    set(Sources ${Sources})
elseif (UNIX)
    set(Sources ${Sources}
        LinuxSerial.cpp
    )
endif()

add_library(Serial ${Sources} ${Headers})
target_compile_features(Serial PUBLIC cxx_std_17)
target_include_directories(Serial PUBLIC ${CMAKE_CURRENT_LIST_DIR})
