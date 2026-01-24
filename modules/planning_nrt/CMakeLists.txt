# ============================================================================
# CMakeLists.txt for planning_nrt module (Modular)
# ============================================================================
cmake_minimum_required(VERSION 3.20)
project(PlanningNRT LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Dependencies Location Setup (Explicit for Windows)
# ============================================================================
if(WIN32)
    set(LIBS_ROOT "C:/libs")
    set(EIGEN3_INCLUDE_DIR "${LIBS_ROOT}/eigen" CACHE PATH "Eigen3 include directory" FORCE)
    set(Eigen3_DIR "${LIBS_ROOT}/eigen/build" CACHE PATH "Eigen3 config directory" FORCE)
    set(orocos_kdl_DIR "${LIBS_ROOT}/orocos-kdl/share/orocos_kdl/cmake" CACHE PATH "KDL config directory" FORCE)
    
    # Also add LIBS_ROOT to prefix path
    list(APPEND CMAKE_PREFIX_PATH "${LIBS_ROOT}/eigen/build" "${LIBS_ROOT}/orocos-kdl")
endif()

# ============================================================================
# External Packages
# ============================================================================
# --- GoogleTest (FetchContent) ---
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(orocos_kdl REQUIRED)

# ============================================================================
# Sub-module Dependencies (if not already defined)
# ============================================================================
if(NOT TARGET kinematics_nrt_lib)
    # We must ensure kinematics_nrt sees EIGEN3_INCLUDE_DIR
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../kinematics_nrt" "${CMAKE_CURRENT_BINARY_DIR}/kinematics_nrt_build")
endif()

if(NOT TARGET hardware_hal_lib)
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../hardware_hal" "${CMAKE_CURRENT_BINARY_DIR}/hardware_hal_build")
endif()

if(NOT TARGET motion_manager_lib)
    # Define motion_manager_lib if not exists
    add_library(motion_manager_lib STATIC
        "${CMAKE_CURRENT_LIST_DIR}/../motion_manager_rt/src/MotionManager.cpp"
    )
    target_include_directories(motion_manager_lib PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/../motion_manager_rt/src"
        "${CMAKE_CURRENT_LIST_DIR}/../trajectory_queue_lf/src"
        "${CMAKE_CURRENT_LIST_DIR}/../../../shared/data_types/src"
        "${CMAKE_CURRENT_LIST_DIR}/../../../shared/logger/src"
        "${CMAKE_CURRENT_LIST_DIR}/../hardware_hal/src"
    )
    target_link_libraries(motion_manager_lib PUBLIC hardware_hal_lib)
endif()

# ============================================================================
# Library: planning_nrt_lib
# ============================================================================
add_library(planning_nrt_lib STATIC
    src/profiles/MotionProfile.cpp
    src/segments/MotionSegment.cpp
    src/segments/TrajectoryInterpolator.cpp
    src/planner/Trajectory.cpp
    src/planner/TrajectoryPlanner.cpp
)

target_include_directories(planning_nrt_lib PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/src"
    "${CMAKE_CURRENT_LIST_DIR}/../motion_manager_rt/src"
    "${CMAKE_CURRENT_LIST_DIR}/../trajectory_queue_lf/src"
)

target_include_directories(planning_nrt_lib PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(planning_nrt_lib PUBLIC
    kinematics_nrt_lib
    motion_manager_lib
    Eigen3::Eigen
)

target_link_libraries(planning_nrt_lib PRIVATE Threads::Threads)

# ============================================================================
# Executable: planning_nrt_test_app
# ============================================================================
add_executable(planning_nrt_test_app
    tests/gtest_trajectory_planner_main.cpp
)

target_include_directories(planning_nrt_test_app PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../kinematics_nrt/tests/mocks"
)

target_link_libraries(planning_nrt_test_app PRIVATE
    planning_nrt_lib
    gtest
    gtest_main
    gmock
    gmock_main
)

message(STATUS "Configured planning_nrt module. Library: 'planning_nrt_lib', Test: 'planning_nrt_test_app'")


