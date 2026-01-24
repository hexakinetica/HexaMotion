# ============================================================================
# CMakeLists.txt for hardware_hal module (Refactored)
# ============================================================================
cmake_minimum_required(VERSION 3.20)
project(HardwareHal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Dependencies
# ============================================================================

# Find shared modules (logger, data_types)
# Assuming they are built as part of a parent project or have been installed.
# For standalone testing, we can add them as subdirectories if needed.
find_package(Threads REQUIRED)

# ============================================================================
# Library: hardware_hal_lib
# This static library contains the HardwareManager and all its drivers.
# It can be linked by other modules like MotionManager.
# ============================================================================
add_library(hardware_hal_lib STATIC
    src/HardwareManager.cpp
    src/drivers/sim/SimDriver.cpp
    src/drivers/udp/UdpDriver.cpp
    # Shared sources
    ../../../shared/data_types/src/DataTypes.cpp
    ../../../shared/logger/src/Logger.cpp
)

target_include_directories(hardware_hal_lib PUBLIC
    # Public headers of this module
    "${CMAKE_CURRENT_LIST_DIR}/src" 
    # Internal headers needed by public ones
    "${CMAKE_CURRENT_LIST_DIR}/src/interface"
    # Shared data types
    "${CMAKE_CURRENT_LIST_DIR}/../../../shared/data_types/src"
    # Shared logger
    "${CMAKE_CURRENT_LIST_DIR}/../../../shared/logger/src"
)

# Link hardware_hal_lib against its dependencies
# These will be inherited by anything that links to hardware_hal_lib
target_link_libraries(hardware_hal_lib PUBLIC
    Threads::Threads
    atomic
)

if(WIN32)
    target_link_libraries(hardware_hal_lib PUBLIC ws2_32)
endif()

# Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

# ============================================================================
# Test Executables
# ============================================================================

# Sim Tests
add_executable(test_hal_sim tests/sim/main.cpp)
target_link_libraries(test_hal_sim PRIVATE hardware_hal_lib GTest::gtest GTest::gtest_main)

# Governor Tests
add_executable(test_hal_governor tests/governor/main.cpp)
target_link_libraries(test_hal_governor PRIVATE hardware_hal_lib GTest::gtest GTest::gtest_main)

# Safety Tests
add_executable(test_hal_safety tests/safety/main.cpp)
target_link_libraries(test_hal_safety PRIVATE hardware_hal_lib GTest::gtest GTest::gtest_main)

# Realtime Tests
add_executable(test_hal_realtime tests/realtime/main.cpp)
target_link_libraries(test_hal_realtime PRIVATE hardware_hal_lib GTest::gtest GTest::gtest_main)

# ============================================================================
# Installation (Optional)
# Defines how to install the library and headers for use by other projects.
# ============================================================================
# install(TARGETS hardware_hal_lib
#     EXPORT HardwareHalTargets
#     ARCHIVE DESTINATION lib
# )
# install(FILES
#     src/HardwareManager.h
#     src/HalTypes.h
#     DESTINATION include/RDT
# )
# install(EXPORT HardwareHalTargets
#     FILE HardwareHalTargets.cmake
#     NAMESPACE RDT::
#     DESTINATION lib/cmake/RDT
# )

message(STATUS "Configured hardware_hal module. Library: 'hardware_hal_lib', Test: 'hardware_hal_test_app'")
