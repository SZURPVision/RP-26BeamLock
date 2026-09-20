add_executable(capture_server src/CaptureServer.cpp)
target_link_libraries(capture_server PUBLIC MVSCamera Core ${OpenCV_LIBS})

add_custom_command(TARGET capture_server POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
			${CMAKE_CURRENT_SOURCE_DIR}/capture_control.html
			$<TARGET_FILE_DIR:capture_server>/capture_control.html
	COMMENT "Copy capture_control.html to runtime directory"
)
