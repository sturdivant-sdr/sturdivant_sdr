cmake_minimum_required(VERSION 3.6)
project(sturdivant_sdr)

# --- CMake Parameters for building executables ---
set(BUILD_TESTS true)

# --- Version Control ---
set(STURDIVANT_SDR_MAJOR_VERSION 0)
set(STURDIVANT_SDR_MINOR_VERSION 0)
set(STURDIVANT_SDR_PATCH_VERSION 1)
set(PROJECT_VERSION ${STURDIVANT_SDR_MAJOR_VERSION}.${STURDIVANT_SDR_MINOR_VERSION}.${STURDIVANT_SDR_PATCH_VERSION})
message(STATUS "${PROJECT_NAME} version ${PROJECT_VERSION}")

# --- Compiler Flags ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Ofast")

# --- Executables build path ---
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/bin)

# --- Dependencies ---
find_package(spdlog REQUIRED)
find_package(eigen3 REQUIRED)
add_subdirectory(src/utils/fftw_wrapper)
add_subdirectory(src/utils/file_operations)
add_subdirectory(src/core)
add_subdirectory(src/constellations)
add_subdirectory(src/tests)
