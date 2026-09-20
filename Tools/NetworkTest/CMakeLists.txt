add_executable(server_test src/ServerTest.cpp)
target_include_directories(server_test PRIVATE include)
target_link_libraries(server_test PRIVATE Core Network)