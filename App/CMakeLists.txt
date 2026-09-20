add_subdirectory(GimbalController)

add_executable(beamlock
	src/Main.cpp
	src/BiasKF.cpp
)
target_include_directories(beamlock PRIVATE include)
target_link_libraries(beamlock PRIVATE Core foxglove_cpp MVSCamera Network TargetDetecter GimbalController)