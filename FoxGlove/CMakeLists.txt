set(CMAKE_CXX_STANDARD 17)

option(SAVE_MCAP "Enable saving MCAP files" ON)

option(USE_FAKE_FOXGLOVE "Use CLI logger instead of Foxglove SDK" OFF)
set(FOXGLOVE_DOWNLOAD_TIMEOUT "300" CACHE STRING "Timeout in seconds for downloading the Foxglove SDK archive")
set(FOXGLOVE_DOWNLOAD_INACTIVITY_TIMEOUT "60" CACHE STRING "Inactivity timeout in seconds for downloading the Foxglove SDK archive")
# 默认从 Foxglove 官方 GitHub Release 下载预编译 SDK（MIT 许可）。
# 国内网络受限时可通过 -DFOXGLOVE_DOWNLOAD_URL=<镜像地址> 覆盖。
set(FOXGLOVE_DOWNLOAD_URL "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.16.2/foxglove-v0.16.2-cpp-x86_64-unknown-linux-gnu.zip" CACHE STRING "URL for the Foxglove SDK archive")
set(FOXGLOVE_DOWNLOAD_HASH "12ccf93169a800496d7e0f4428127e92cb00e1862a3e609f141a2aaae0c8946f" CACHE STRING "SHA256 hash for the Foxglove SDK archive")
option(FOXGLOVE_DOWNLOAD_SHOW_PROGRESS "Show download progress while fetching the Foxglove SDK archive" ON)

if (SAVE_MCAP)
    add_definitions(-DSAVE_MCAP)
    message(STATUS "MCAP saving enabled")
else()
    message(STATUS "MCAP saving disabled")
endif()

if (USE_FAKE_FOXGLOVE)
    add_library(foxglove_cpp SHARED src/FoxGloveServerFake.cpp)
    target_include_directories(foxglove_cpp PUBLIC include)
    target_link_libraries(foxglove_cpp PUBLIC Core)
    target_compile_definitions(foxglove_cpp PUBLIC USE_FAKE_FOXGLOVE=1)
else()
    include(FetchContent)

    set(FOXGLOVE_ARCHIVE_DIR "${CMAKE_CURRENT_BINARY_DIR}/_foxglove_archive")
    set(FOXGLOVE_ARCHIVE_PATH "${FOXGLOVE_ARCHIVE_DIR}/foxglove-v0.16.2-cpp-x86_64-unknown-linux-gnu.zip")
    file(MAKE_DIRECTORY "${FOXGLOVE_ARCHIVE_DIR}")

    set(FOXGLOVE_ARCHIVE_NEEDS_DOWNLOAD ON)
    if(EXISTS "${FOXGLOVE_ARCHIVE_PATH}")
        file(SHA256 "${FOXGLOVE_ARCHIVE_PATH}" FOXGLOVE_ARCHIVE_CURRENT_HASH)
        if(FOXGLOVE_ARCHIVE_CURRENT_HASH STREQUAL FOXGLOVE_DOWNLOAD_HASH)
            set(FOXGLOVE_ARCHIVE_NEEDS_DOWNLOAD OFF)
        endif()
    endif()

    if(FOXGLOVE_ARCHIVE_NEEDS_DOWNLOAD)
        message(STATUS "Downloading Foxglove SDK archive with progress...")
        file(DOWNLOAD
            "${FOXGLOVE_DOWNLOAD_URL}"
            "${FOXGLOVE_ARCHIVE_PATH}"
            EXPECTED_HASH SHA256=${FOXGLOVE_DOWNLOAD_HASH}
            SHOW_PROGRESS
            TIMEOUT ${FOXGLOVE_DOWNLOAD_TIMEOUT}
            INACTIVITY_TIMEOUT ${FOXGLOVE_DOWNLOAD_INACTIVITY_TIMEOUT}
            STATUS FOXGLOVE_DOWNLOAD_STATUS
        )
        list(GET FOXGLOVE_DOWNLOAD_STATUS 0 FOXGLOVE_DOWNLOAD_STATUS_CODE)
        list(GET FOXGLOVE_DOWNLOAD_STATUS 1 FOXGLOVE_DOWNLOAD_STATUS_MESSAGE)
        if(NOT FOXGLOVE_DOWNLOAD_STATUS_CODE EQUAL 0)
            file(REMOVE "${FOXGLOVE_ARCHIVE_PATH}")
            message(FATAL_ERROR "Failed to download Foxglove SDK archive: ${FOXGLOVE_DOWNLOAD_STATUS_MESSAGE}")
        endif()
    endif()

    FetchContent_Declare(
        foxglove
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        URL "${FOXGLOVE_ARCHIVE_PATH}"
    )
    FetchContent_MakeAvailable(foxglove)

    add_library(foxglove_cpp SHARED src/FoxGloveServer.cpp)
    target_include_directories(foxglove_cpp PUBLIC include)
    target_link_libraries(foxglove_cpp PUBLIC ${OpenCV_LIBS} Core)
    target_include_directories(foxglove_cpp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../Communication/Network/include)

    target_include_directories(foxglove_cpp PUBLIC ${foxglove_SOURCE_DIR}/include)
    target_include_directories(foxglove_cpp PUBLIC ${foxglove_SOURCE_DIR}/include/foxglove)

    file(GLOB FOXGLOVE_SOURCES CONFIGURE_DEPENDS
        "${foxglove_SOURCE_DIR}/src/*.cpp"
        "${foxglove_SOURCE_DIR}/src/server/*.cpp"
    )

    target_sources(foxglove_cpp PRIVATE ${FOXGLOVE_SOURCES})

    target_link_libraries(foxglove_cpp PRIVATE ${foxglove_SOURCE_DIR}/lib/libfoxglove.a)
endif()