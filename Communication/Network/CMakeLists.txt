add_library(Network SHARED src/Network.cpp)
target_link_libraries(Network PUBLIC Core foxglove_cpp)
target_include_directories(Network PUBLIC include)