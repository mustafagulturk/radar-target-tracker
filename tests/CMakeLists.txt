# Unit and integration tests (GoogleTest).
#
# Uses an installed GoogleTest when available (e.g. libgtest-dev) and falls
# back to downloading a pinned release otherwise.

find_package(GTest QUIET)

if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    # Keep GoogleTest's runtime library choice consistent with ours on MSVC.
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # Treat GoogleTest headers as system headers so our strict warning flags
    # (and -Werror) do not apply to third-party code.
    foreach(gtest_target gtest gtest_main)
        get_target_property(gtest_includes ${gtest_target} INTERFACE_INCLUDE_DIRECTORIES)
        set_target_properties(${gtest_target} PROPERTIES
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${gtest_includes}")
    endforeach()
    if(NOT TARGET GTest::gtest_main)
        add_library(GTest::gtest_main ALIAS gtest_main)
    endif()
endif()

include(GoogleTest)

add_executable(radar_tests
    test_matrix.cpp
    test_coordinates.cpp
    test_kalman_filter.cpp
    test_protocol.cpp
    test_thread_safe_queue.cpp
    test_track_manager.cpp
    test_sim.cpp
    test_pipeline.cpp)
target_link_libraries(radar_tests PRIVATE radar_sim GTest::gtest_main)
radar_set_project_options(radar_tests)

gtest_discover_tests(radar_tests
    DISCOVERY_TIMEOUT 30
    PROPERTIES TIMEOUT 60)
