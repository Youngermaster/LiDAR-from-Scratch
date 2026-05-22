# =====================================================================
# 01_hello_sdk: connect, query INFO and HEALTH, disconnect.
#
# This CMakeLists deliberately keeps the rplidar_sdk integration small
# and explicit so the C++ build is readable for someone learning CMake
# alongside the experiment.
# =====================================================================

add_executable(hello_sdk main.cpp)

target_include_directories(hello_sdk PRIVATE
    "${RPLIDAR_SDK_INCLUDE_DIR}"
)

# We compile the SDK's sources directly into this target rather than
# building a separate static library. This avoids the SDK's own
# makefile-based build and keeps everything in CMake. The exact source
# list comes from the SDK's `sdk/src` tree.
file(GLOB RPLIDAR_SDK_COMMON_SOURCES
    "${RPLIDAR_SDK_SRC_DIR}/*.cpp"
    "${RPLIDAR_SDK_SRC_DIR}/hal/*.cpp"
)
if(APPLE OR UNIX)
    file(GLOB RPLIDAR_SDK_PLATFORM_SOURCES
        "${RPLIDAR_SDK_SRC_DIR}/arch/linux/*.cpp"
    )
elseif(WIN32)
    file(GLOB RPLIDAR_SDK_PLATFORM_SOURCES
        "${RPLIDAR_SDK_SRC_DIR}/arch/win32/*.cpp"
    )
endif()

if(RPLIDAR_SDK_COMMON_SOURCES)
    target_sources(hello_sdk PRIVATE
        ${RPLIDAR_SDK_COMMON_SOURCES}
        ${RPLIDAR_SDK_PLATFORM_SOURCES}
    )
endif()

if(UNIX AND NOT APPLE)
    target_link_libraries(hello_sdk PRIVATE pthread)
endif()
if(APPLE)
    target_link_libraries(hello_sdk PRIVATE pthread)
endif()
