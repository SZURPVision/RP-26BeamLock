add_library(TargetDetecter SHARED
	src/TargetDetecter.cpp
	src/LightBlob.cpp
	src/LightRow.cpp
	src/Target.cpp
)
target_link_libraries(TargetDetecter PUBLIC ${OpenCV_LIBS} Core foxglove_cpp)
target_include_directories(TargetDetecter PUBLIC include)