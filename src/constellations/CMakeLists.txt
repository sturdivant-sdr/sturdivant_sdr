cmake_minimum_required(VERSION 3.6)
project(sturdivant_sdr_constellations)

# # --- Version Control ---
# set(STURDIVANT_SDR_CONSTELLATIONS_MAJOR_VERSION 0)
# set(STURDIVANT_SDR_CONSTELLATIONS_MINOR_VERSION 0)
# set(STURDIVANT_SDR_CONSTELLATIONS_PATCH_VERSION 1)
# set(PROJECT_VERSION ${STURDIVANT_SDR_CONSTELLATIONS_MAJOR_VERSION}.${STURDIVANT_SDR_CONSTELLATIONS_MINOR_VERSION}.${STURDIVANT_SDR_CONSTELLATIONS_PATCH_VERSION})
# message(STATUS "${PROJECT_NAME} version ${PROJECT_VERSION}")

# # --- Compiler Flags ---
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS "-Wall -Wextra")

# set(CMAKE_CXX_FLAGS_DEBUG "-g")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Ofast")

# # --- Executables build path ---
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/bin)

# --- Dependencies ---
find_package(spdlog REQUIRED)
find_package(eigen3 REQUIRED)

# add_subdirectory(src/utils/fftw_wrapper)
# add_subdirectory(src/utils/file_operations)

# --- Create Library ---
set(STURDIVANT_SDR_CONSTELLATIONS_HDRS
    include/constellations/gps/gps_constants.hpp
    include/constellations/gps/gps_ephemeris.hpp
    include/constellations/gps/gps_l1ca_channel.hpp
    include/constellations/gps/gps_lnav_parser.hpp
)
set(STURDIVANT_SDR_CONSTELLATIONS_SRCS
    src/gps_ephemeris.cpp
    src/gps_l1ca_channel.cpp
    src/gps_lnav_parser.cpp
)

add_library(${PROJECT_NAME} ${STURDIVANT_SDR_CONSTELLATIONS_HDRS} ${STURDIVANT_SDR_CONSTELLATIONS_SRCS})
target_include_directories(
    ${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
)
target_link_libraries(
    ${PROJECT_NAME} PUBLIC
    spdlog::spdlog
    Eigen3::Eigen
    sturdivant_sdr_core
)

# --- Build Executables ---
if(BUILD_TESTS)
endif()