add_library(Core SHARED src/Config.cpp src/ConfigNode.cpp src/TimeUtils.cpp)
target_link_libraries(Core PUBLIC yaml-cpp ${OpenCV_LIBS} Eigen3::Eigen)
target_include_directories(Core PUBLIC include)