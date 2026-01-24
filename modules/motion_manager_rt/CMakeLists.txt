# ============================================================================
# CMakeLists.txt for motion_manager_rt module (Refactored)
# ============================================================================
cmake_minimum_required(VERSION 3.20)
project(MotionManager_Test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Dependencies
# ============================================================================

# Add hardware_hal as a subdirectory to make hardware_hal_lib available
if(NOT TARGET hardware_hal_lib)
    add_subdirectory(../hardware_hal ../hardware_hal/build_for_mm)
endif()

# Google Test (FetchContent)
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

find_package(Threads REQUIRED)

# ============================================================================
# Executable: motion_manager_test_app
# The comprehensive GTest suite for this module.
# ============================================================================
add_executable(motion_manager_test_app
    tests/gtest_motion_manager_main.cpp
    src/MotionManager.cpp
)

target_include_directories(motion_manager_test_app PRIVATE
    # Headers for this module
    "src"
    # Dependencies
    "${CMAKE_CURRENT_LIST_DIR}/../trajectory_queue_lf/src"
    "${CMAKE_CURRENT_LIST_DIR}/../../../shared/data_types/src"
    "${CMAKE_CURRENT_LIST_DIR}/../../../shared/logger/src"
)

# Link the test app to its dependencies:
target_link_libraries(motion_manager_test_app PRIVATE
    hardware_hal_lib
    gtest
    gtest_main
    Threads::Threads
)

if(WIN32)
    target_link_libraries(motion_manager_test_app PRIVATE ws2_32)
endif()

message(STATUS "Configured motion_manager_rt module. Test: 'motion_manager_test_app'")
