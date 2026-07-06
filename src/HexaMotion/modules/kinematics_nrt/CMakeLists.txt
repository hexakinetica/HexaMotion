
# ============================================================================
# CMakeLists.txt for kinematics_nrt module (Refactored)
# ============================================================================
cmake_minimum_required(VERSION 3.20)
project(KinematicsNRT LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT DEFINED HEXA_THIRD_PARTY_DIR)
    set(HEXA_THIRD_PARTY_DIR "${CMAKE_SOURCE_DIR}/third_party")
endif()
if(NOT DEFINED HEXA_THIRD_PARTY_COMMON_DIR)
    set(HEXA_THIRD_PARTY_COMMON_DIR "${HEXA_THIRD_PARTY_DIR}/common")
endif()
if(NOT DEFINED HEXA_THIRD_PARTY_WIN_DIR)
    set(HEXA_THIRD_PARTY_WIN_DIR "${HEXA_THIRD_PARTY_DIR}/win")
endif()

# ============================================================================
# Dependencies
# ============================================================================

# --- GoogleTest (FetchContent) ---
if(NOT TARGET GTest::gtest)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  )
  # For Windows:
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

find_package(Threads REQUIRED)
# find_package(Eigen3 REQUIRED NO_MODULE)
set(EIGEN3_INCLUDE_DIR "${HEXA_THIRD_PARTY_COMMON_DIR}/eigen")

set(orocos_kdl_DIR "${HEXA_THIRD_PARTY_WIN_DIR}/orocos-kdl/share/orocos_kdl/cmake")
find_package(orocos_kdl REQUIRED)

# ============================================================================
# Library: kinematics_nrt_lib
# ============================================================================
add_library(kinematics_nrt_lib STATIC
    src/kinematic_solver/KdlKinematicSolver.cpp
    src/frame_processor/FrameTransformer.cpp
    # Include shared components sources if they are not built as separate libs here
    ${HEXA_SDK_DIR}/data_types/src/DataTypes.cpp
    ${HEXA_SDK_DIR}/logger/src/Logger.cpp
)

target_include_directories(kinematics_nrt_lib PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/src"
    "${CMAKE_CURRENT_LIST_DIR}/src/kinematic_solver"
    "${CMAKE_CURRENT_LIST_DIR}/src/frame_processor"
    "${HEXA_SDK_DIR}/data_types/src"
    "${HEXA_SDK_DIR}/logger/src"
    "${CMAKE_CURRENT_LIST_DIR}/include"
    "${EIGEN3_INCLUDE_DIR}"
    ${orocos_kdl_INCLUDE_DIRS}
)

target_link_libraries(kinematics_nrt_lib PRIVATE
    Threads::Threads
    ${orocos_kdl_LIBRARIES}
    robot_model
)

# ============================================================================
# Executable: kinematics_nrt_test_app
# ============================================================================
add_executable(kinematics_nrt_test_app
    tests/gtest_kinematic_solver_main.cpp
    tests/gtest_frame_transformer_main.cpp
)

target_include_directories(kinematics_nrt_test_app PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/tests"
)

target_link_libraries(kinematics_nrt_test_app PRIVATE
    kinematics_nrt_lib
    robot_model
    gtest
    gtest_main
)

add_executable(kinematics_nrt_ik_isolated_tests
    tests/gtest_ik_isolated.cpp
)

target_link_libraries(kinematics_nrt_ik_isolated_tests PRIVATE
    kinematics_nrt_lib
    robot_model
    gtest
    gtest_main
)

message(STATUS "Configured kinematics_nrt module. Library: 'kinematics_nrt_lib', Test: 'kinematics_nrt_test_app'")
