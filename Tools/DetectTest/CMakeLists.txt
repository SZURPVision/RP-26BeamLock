add_executable(detect_test
    src/DetectTest.cpp
)

target_link_libraries(detect_test PUBLIC TargetDetecter MVSCamera Core ${OpenCV_LIBS})

add_executable(target_detecter_test
    src/TargetDetecterTest.cpp
)

target_link_libraries(target_detecter_test PUBLIC TargetDetecter MVSCamera Core ${OpenCV_LIBS})
