add_executable(gimbal_controller_test
    src/GimbalControllerTest.cpp
)

target_link_libraries(gimbal_controller_test PUBLIC GimbalController Core)
