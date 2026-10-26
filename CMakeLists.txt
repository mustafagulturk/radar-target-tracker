cmake_minimum_required(VERSION 3.16)

project(radar_target_tracker
    VERSION 1.0.0
    DESCRIPTION "Radar target tracking simulator: UDP plots, multithreaded Kalman tracking, Qt PPI display"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(RADAR_BUILD_GUI "Build the Qt radar display" ON)
option(RADAR_BUILD_TESTS "Build the GoogleTest unit tests" ON)
option(RADAR_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(RADAR_ENABLE_SANITIZERS "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(RADAR_ENABLE_TSAN "Build with ThreadSanitizer" OFF)

include(cmake/CompilerOptions.cmake)

find_package(Threads REQUIRED)

# --- Core library: math, filtering, tracking, protocol, networking ----------
add_library(radar_core
    src/core/coordinates.cpp
    src/core/kalman_filter.cpp
    src/core/protocol.cpp
    src/core/track_manager.cpp
    src/core/tracking_pipeline.cpp
    src/core/udp_socket.cpp)
target_include_directories(radar_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(radar_core PUBLIC Threads::Threads)
radar_set_project_options(radar_core)

# --- Simulation library: targets, sensor model, scenarios --------------------
add_library(radar_sim
    src/sim/radar_sensor.cpp
    src/sim/scenario.cpp
    src/sim/target.cpp)
target_link_libraries(radar_sim PUBLIC radar_core)
radar_set_project_options(radar_sim)

# --- Applications ------------------------------------------------------------
add_executable(radar_simulator apps/radar_simulator/main.cpp)
target_link_libraries(radar_simulator PRIVATE radar_sim)
radar_set_project_options(radar_simulator)

add_executable(tracker_cli apps/tracker_cli/main.cpp)
target_link_libraries(tracker_cli PRIVATE radar_core)
radar_set_project_options(tracker_cli)

add_executable(tracker_benchmark apps/tracker_benchmark/main.cpp)
target_link_libraries(tracker_benchmark PRIVATE radar_sim)
radar_set_project_options(tracker_benchmark)

if(RADAR_BUILD_GUI)
    add_subdirectory(apps/radar_display)
endif()

if(RADAR_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
