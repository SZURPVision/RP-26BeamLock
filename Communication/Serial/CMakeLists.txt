add_subdirectory(CRCCheck)

add_library(Serial SHARED src/Serial.cpp)
target_link_libraries(Serial PUBLIC Core foxglove_cpp)
target_link_libraries(Serial PRIVATE CRCCheck)
target_include_directories(Serial PUBLIC include)