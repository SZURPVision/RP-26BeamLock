add_executable(camera_tune src/CameraTune.cpp)
target_link_libraries(camera_tune PUBLIC MVSCamera Core ${OpenCV_LIBS})

add_executable(camera_capture src/CameraCapture.cpp)
target_link_libraries(camera_capture PUBLIC MVSCamera Core ${OpenCV_LIBS} Serial)

add_executable(camera_calibration src/CameraCalibration.cpp)
target_link_libraries(camera_calibration PUBLIC ${OpenCV_LIBS})

add_subdirectory(CaptureServer)