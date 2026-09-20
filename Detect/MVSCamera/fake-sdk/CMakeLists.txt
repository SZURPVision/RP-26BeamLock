# 假海康 MVS SDK：仅供 -DUSE_FAKE_MVS=ON 的编译/链接验证使用。
#
# 这里用 STATIC 而不是 SHARED，是为了在物理上无法冒充官方闭源库：
# 不会产出 libMvCameraControl.so，避免有人误把它当成真 SDK 装进 /opt/MVS。
add_library(MvCameraControl STATIC MvCameraControl_stub.cpp)
add_library(MVS::MvCameraControl ALIAS MvCameraControl)

target_include_directories(MvCameraControl PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
set_target_properties(MvCameraControl PROPERTIES POSITION_INDEPENDENT_CODE ON)
