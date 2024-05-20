cmake_minimum_required(VERSION 3.6)
project(sturdivant_sdr_tests)

# --- Build Executables ---
if(BUILD_TESTS)
    # set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/bin)
    add_executable(test_binary test_binary.cpp)
    target_link_libraries(test_binary PUBLIC)

    add_executable(test_multi_threading test_multi_threading.cpp)
    target_link_libraries(test_multi_threading PUBLIC)

    add_executable(test_channel test_channel.cpp)
    target_link_libraries(test_channel
        PUBLIC

        # sturdivant_sdr_core
        sturdivant_sdr_constellations
        file_operations
    )

    list(APPEND FILE_OPERATIONS_TESTS
        test_binary
        test_multi_threading
        test_channel
    )
    install(
        TARGETS ${FILE_OPERATIONS_TESTS}
        DESTINATION bin
    )
endif()