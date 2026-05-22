# =====================================================================
# Top-level CMakeLists for the C++ track.
#
# This file does three things:
#   1) Sets the project-wide C++ standard.
#   2) Adds the rplidar_sdk submodule as a library target.
#   3) Adds each experiment subdirectory.
#
# Each experiment is a self-contained CMake subproject with its own
# CMakeLists.txt; this file just wires them all together so a single
# `cmake --build build` builds everything.
# =====================================================================

cmake_minimum_required(VERSION 3.20)
project(lidar_from_scratch_cpp LANGUAGES CXX)

# Force C++17 across the board. Individual experiments can opt into
# C++20 by setting CMAKE_CXX_STANDARD locally.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Sensible default if no -DCMAKE_BUILD_TYPE was supplied.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Default build type" FORCE)
endif()

# ---------------------------------------------------------------------
# rplidar_sdk submodule.
#
# The SDK ships its own makefile-based build; here we expose its source
# tree directly to our experiments by adding the include directory.
# Experiments link against the SDK's static library object files via
# their own CMakeLists. We keep this loose-coupled so the SDK can be
# updated independently.
# ---------------------------------------------------------------------
set(RPLIDAR_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/rplidar_sdk")
if(NOT EXISTS "${RPLIDAR_SDK_DIR}/sdk/include")
    message(WARNING
        "rplidar_sdk submodule not found at ${RPLIDAR_SDK_DIR}.\n"
        "Run from the repo root:\n"
        "    git submodule add https://github.com/Slamtec/rplidar_sdk "
            "cpp/third_party/rplidar_sdk\n"
        "    git submodule update --init --recursive\n"
        "Experiments will not build until this is done.")
endif()

# Expose paths as CMake variables that the experiment subdirs can use.
set(RPLIDAR_SDK_INCLUDE_DIR "${RPLIDAR_SDK_DIR}/sdk/include")
set(RPLIDAR_SDK_SRC_DIR "${RPLIDAR_SDK_DIR}/sdk/src")

# ---------------------------------------------------------------------
# Experiments.
#
# Add each as a subdirectory below. Keep this list ordered by experiment
# number so the build output is easy to scan.
# ---------------------------------------------------------------------
add_subdirectory(experiments/01_hello_sdk)
add_subdirectory(experiments/02_scan_print)
