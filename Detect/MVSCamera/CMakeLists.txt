add_library(MVSCamera SHARED
    src/Camera.cpp
    src/CameraException.cpp
    src/CameraInfoList.cpp
    src/CameraManager.cpp
    src/HandleManager.cpp
)
target_include_directories(MVSCamera PUBLIC include)
target_link_libraries(MVSCamera PUBLIC ${OpenCV_LIBS})

# 海康机器人 MVS SDK（MvCameraControl）为闭源商业 SDK，本仓库不包含也不得再分发。
#
# USE_FAKE_MVS=OFF（默认，真实部署）：
#   请从海康机器人官网下载安装 MVS，然后：
#     - 使用默认安装路径 /opt/MVS，或
#     - 通过 -DMVS_ROOT=<安装路径> 指定 SDK 位置
# USE_FAKE_MVS=ON（仅编译验证）：
#   使用本目录 fake-sdk/ 下的空实现替身，无需安装 MVS。
option(USE_FAKE_MVS "使用 fake-sdk/ 下的假 MVS SDK 做编译验证（不连接真实相机）" OFF)

if(USE_FAKE_MVS)
    message(WARNING
        "USE_FAKE_MVS=ON：正在使用假 MVS SDK（空实现），只能做编译验证，"
        "运行时枚举不到任何相机，请勿用于真实部署。")

    add_subdirectory(fake-sdk)
    target_link_libraries(MVSCamera PUBLIC MVS::MvCameraControl)
else()
    set(MVS_ROOT "/opt/MVS" CACHE PATH "海康 MVS SDK 安装路径")

    find_path(MVS_INCLUDE_DIR
        NAMES MvCameraControl.h
        PATHS "${MVS_ROOT}/include"
        NO_DEFAULT_PATH
    )
    find_library(MVS_LIBRARY
        NAMES MvCameraControl
        PATHS "${MVS_ROOT}/lib/64" "${MVS_ROOT}/lib"
        NO_DEFAULT_PATH
    )

    if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
        message(FATAL_ERROR
            "未找到海康 MVS SDK（MvCameraControl.h / libMvCameraControl.so）。\n"
            "请先安装 MVS SDK（见 README 的依赖说明），或用 -DMVS_ROOT=<安装路径> 指定其位置。\n"
            "若只是想做编译验证，可用 -DUSE_FAKE_MVS=ON 使用仓库内置的假 SDK。")
    endif()

    target_include_directories(MVSCamera PUBLIC ${MVS_INCLUDE_DIR})
    target_link_libraries(MVSCamera PUBLIC ${MVS_LIBRARY})
endif()
