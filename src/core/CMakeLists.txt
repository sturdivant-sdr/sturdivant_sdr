cmake_minimum_required(VERSION 3.6)
project(sturdivant_sdr_core)

# --- CMake Parameters for building executables ---
# set(BUILD_TESTS true)

# # --- Version Control ---
# set(STURDIVANT_SDR_CORE_MAJOR_VERSION 0)
# set(STURDIVANT_SDR_CORE_MINOR_VERSION 0)
# set(STURDIVANT_SDR_CORE_PATCH_VERSION 1)
# set(PROJECT_VERSION ${STURDIVANT_SDR_CORE_MAJOR_VERSION}.${STURDIVANT_SDR_CORE_MINOR_VERSION}.${STURDIVANT_SDR_CORE_PATCH_VERSION})
# message(STATUS "${PROJECT_NAME} version ${PROJECT_VERSION}")

# # --- Compiler Flags ---
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS "-Wall -Wextra")

# set(CMAKE_CXX_FLAGS_DEBUG "-g")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Ofast")

# --- Dependencies ---
find_package(spdlog REQUIRED)
find_package(eigen3 REQUIRED)

# add_subdirectory(src/utils/fftw_wrapper)
# add_subdirectory(src/utils/file_operations)

# --- Create Library ---
set(STURDIVANT_SDR_CORE_HDRS
    include/core/algorithms/binary_operations.hpp
    include/core/algorithms/lock_detectors.hpp
    include/core/algorithms/statistics.hpp
    include/core/algorithms/tracking_discriminators.hpp
    include/core/blocks/pcps_acquire.hpp
    include/core/blocks/tracking.hpp
    include/core/param/math_constants.hpp
    include/core/param/structures.hpp
)
set(STURDIVANT_SDR_CORE_SRCS
    src/binary_operations.cpp
    src/lock_detectors.cpp
    src/tracking_discriminators.cpp
    src/pcps_acquire.cpp
    src/tracking.cpp
)

add_library(${PROJECT_NAME} ${STURDIVANT_SDR_CORE_SRCS} ${STURDIVANT_SDR_CORE_SRCS})
target_include_directories(
    ${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
)
target_link_libraries(
    ${PROJECT_NAME} PUBLIC
    spdlog::spdlog
    Eigen3::Eigen
    fftw_wrapper
    file_operations
)

# --- Build Executables ---
if(BUILD_TESTS)
    # # set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/bin)
    # add_executable(test_binary tests/test_binary.cpp)
    # add_executable(test_multi_threading tests/test_multi_threading.cpp)

    # # add_executable(test_channel coretests/test_channel.cpp)
    # target_link_libraries(test_binary PUBLIC ${PROJECT_NAME})
    # target_link_libraries(test_multi_threading PUBLIC ${PROJECT_NAME})

    # # target_link_libraries(test_channel PUBLIC ${PROJECT_NAME})
    # list(APPEND FILE_OPERATIONS_TESTS
    # test_binary
    # test_multi_threading

    # # test_channel
    # )
    # install(
    # TARGETS ${FILE_OPERATIONS_TESTS}
    # DESTINATION bin
    # )
endif()