# =====================================================================
# 02_scan_print: start the motor, stream scan samples, stop cleanly.
# =====================================================================

add_executable(scan_print main.cpp)

target_include_directories(scan_print PRIVATE
    "${RPLIDAR_SDK_INCLUDE_DIR}"
)

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
    target_sources(scan_print PRIVATE
        ${RPLIDAR_SDK_COMMON_SOURCES}
        ${RPLIDAR_SDK_PLATFORM_SOURCES}
    )
endif()

if(UNIX)
    target_link_libraries(scan_print PRIVATE pthread)
endif()
