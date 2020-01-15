set(Headers
    Serial.hpp
)
set(Sources 
    LinuxSerial.cpp 
)

add_library(${LibName} ${Sources} ${Headers})
target_compile_features(${LibName} PRIVATE cxx_std_17)

install(TARGETS ${LibName} DESTINATION lib)
install(FILES ${Headers} DESTINATION include/Henry)
