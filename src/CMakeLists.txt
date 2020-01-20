set(Headers
    Serial.hpp
)
set(Sources 
    LinuxSerial.cpp
)

add_library(Serial ${Sources} ${Headers})
target_compile_features(Serial PUBLIC cxx_std_17)
target_include_directories(Serial PUBLIC ${CMAKE_CURRENT_LIST_DIR})
