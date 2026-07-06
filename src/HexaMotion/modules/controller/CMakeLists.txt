# ============================================================================
# CMakeLists.txt for controller module
# ============================================================================
cmake_minimum_required(VERSION 3.20)
project(RobotController LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Dependencies Setup (Standalone Build Support)
# ============================================================================

if(NOT DEFINED HEXA_THIRD_PARTY_DIR)
    set(HEXA_THIRD_PARTY_DIR "${CMAKE_SOURCE_DIR}/third_party")
endif()
if(NOT DEFINED HEXA_THIRD_PARTY_COMMON_DIR)
    set(HEXA_THIRD_PARTY_COMMON_DIR "${HEXA_THIRD_PARTY_DIR}/common")
endif()
if(NOT DEFINED HEXA_THIRD_PARTY_WIN_DIR)
    set(HEXA_THIRD_PARTY_WIN_DIR "${HEXA_THIRD_PARTY_DIR}/win")
endif()

if(WIN32)
    set(COMMON_LIBS_ROOT "${HEXA_THIRD_PARTY_COMMON_DIR}")
    set(WIN_LIBS_ROOT "${HEXA_THIRD_PARTY_WIN_DIR}")
    # Set Eigen3 and Orocos KDL paths if not already set
    if(NOT DEFINED EIGEN3_INCLUDE_DIR)
        set(EIGEN3_INCLUDE_DIR "${COMMON_LIBS_ROOT}/eigen" CACHE PATH "Eigen3 include directory" FORCE)
        set(Eigen3_DIR "${COMMON_LIBS_ROOT}/eigen/build" CACHE PATH "Eigen3 config directory" FORCE)
    endif()
    if(NOT DEFINED orocos_kdl_DIR)
        set(orocos_kdl_DIR "${WIN_LIBS_ROOT}/orocos-kdl/share/orocos_kdl/cmake" CACHE PATH "KDL config directory" FORCE)
    endif()
    
    list(APPEND CMAKE_PREFIX_PATH "${COMMON_LIBS_ROOT}/eigen/build" "${WIN_LIBS_ROOT}/orocos-kdl")
endif()

find_package(Threads REQUIRED)

# --- GoogleTest (FetchContent) ---
# Check if GTest is already available (e.g. from parent)
if(NOT TARGET GTest::gtest)
    include(FetchContent)
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

# ============================================================================
# Module Dependencies (Workarounds for Standalone Build)
# ============================================================================

# 1. Hardware HAL
if(NOT TARGET hardware_hal_lib)
    message(STATUS "Importing hardware_hal module...")
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../hardware_hal" "${CMAKE_CURRENT_BINARY_DIR}/hardware_hal_build")
endif()

# 2. Kinematics NRT
if(NOT TARGET kinematics_nrt_lib)
    message(STATUS "Importing kinematics_nrt module...")
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../kinematics_nrt" "${CMAKE_CURRENT_BINARY_DIR}/kinematics_nrt_build")
endif()

# 3. Planning NRT
if(NOT TARGET planning_nrt_lib)
    message(STATUS "Importing planning_nrt module...")
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../planning_nrt" "${CMAKE_CURRENT_BINARY_DIR}/planning_nrt_build")
endif()

# 4. Motion Manager (Workaround: Define missing library target)
if(NOT TARGET motion_manager_lib)
    message(STATUS "Defining workaround target for motion_manager_lib")
    add_library(motion_manager_lib STATIC
        "${CMAKE_CURRENT_LIST_DIR}/../motion_manager_rt/src/MotionManager.cpp"
    )
    target_include_directories(motion_manager_lib PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/../motion_manager_rt/src"
        "${CMAKE_CURRENT_LIST_DIR}/../trajectory_queue_lf/src"
        "${HEXA_SDK_DIR}/data_types/src"
        "${HEXA_SDK_DIR}/logger/src"
        "${CMAKE_CURRENT_LIST_DIR}/../hardware_hal/src"
    )
    # Link dependencies required by MotionManager
    target_link_libraries(motion_manager_lib PUBLIC hardware_hal_lib)
    if(WIN32)
        # winmm: timeBeginPeriod/timeEndPeriod (RT cycle pacing, REQ_rt_cycle_pacing).
        target_link_libraries(motion_manager_lib PUBLIC winmm)
    endif()
endif()

# 5. Shared Components (Logger, DataTypes, StateData, RDT Protocol, Persistence)
# We compile StateData sources locally because they are not provided by other libs
if(NOT TARGET state_data_lib)
    add_library(state_data_lib STATIC
        "${HEXA_SDK_DIR}/state_data/src/RobotState.cpp"
        "${HEXA_SDK_DIR}/state_data/src/ClientState.cpp"
        "${HEXA_SDK_DIR}/persistence/src/PersistenceManager.cpp"
        "${HEXA_SDK_DIR}/persistence/src/FileManager.cpp"
    )
    target_include_directories(state_data_lib PUBLIC 
        "${HEXA_SDK_DIR}/data_types/src"
        "${HEXA_SDK_DIR}/logger/src"
        "${HEXA_SDK_DIR}/state_data/src"
        "${HEXA_SDK_DIR}/rdt_protocol/src"
        "${HEXA_SDK_DIR}/rdt_protocol"
        "${HEXA_SDK_DIR}/persistence/src"
        "${HEXA_SDK_DIR}/network_driver/src"
        "${HEXA_SDK_DIR}/rdt_bridge/src"
    )
endif()

# We still use an interface for others if needed, but for now we'll use state_data_lib
if(NOT TARGET state_data_interface)
    add_library(state_data_interface INTERFACE)
    target_link_libraries(state_data_interface INTERFACE state_data_lib)
endif()

if(NOT TARGET logger_interface)
    add_library(logger_interface INTERFACE)
    target_include_directories(logger_interface INTERFACE "${HEXA_SDK_DIR}/logger/src")
endif()

# ============================================================================
# Library: controller_lib
# ============================================================================
add_library(controller_lib STATIC
    src/RobotController.cpp
    # Program sequencer: the pure flat-program interpreter driven by RobotController (loops/branches/
    # waits). Compiled directly (single .cpp, header-only deps on rdt_protocol/data_types already on
    # the include path via state_data). NOT the module's own lib target, to avoid pulling its bundled
    # DataTypes.cpp (already provided by the data_types build in the full tree).
    "${CMAKE_CURRENT_LIST_DIR}/../program_sequencer/src/ProgramSequencer.cpp"
)

target_include_directories(controller_lib PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/src"
    "${CMAKE_CURRENT_LIST_DIR}/../program_sequencer/src"
)

# Link against all subsystems
target_link_libraries(controller_lib PUBLIC
    hardware_hal_lib
    motion_manager_lib
    planning_nrt_lib
    kinematics_nrt_lib
    state_data_interface
    logger_interface
    Threads::Threads
)

# ============================================================================
# Executable: controller_test_app
# ============================================================================
add_executable(controller_test_app
    test/gtest_robot_controller_main.cpp
)

target_link_libraries(controller_test_app PRIVATE
    controller_lib
    GTest::gtest
    GTest::gtest_main
)

message(STATUS "Configured controller module. Library: 'controller_lib', Test: 'controller_test_app'")
