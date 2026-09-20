add_executable(serial_test src/SerialTest.cpp)
target_link_libraries(serial_test PRIVATE Serial Core)

add_executable(serial_simulate src/SerialSimulate.cpp)
target_link_libraries(serial_simulate PRIVATE CRCCheck)
