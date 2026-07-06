# ============================================================================
# CMakeLists.txt for the program_sequencer module (standalone, pre-integration).
#
# Pure controller-side interpreter of the authored flat program (loops/branches/waits). No Qt, no
# hardware, no threads of its own — it depends only on the shared protocol/data headers. Built and
# tested in isolation; integration into HexaMotion (RobotController) is a separate, later step.
#
# Standalone build (from this folder):
#   cmake -S . -B build
#   cmake --build build
#   ./build/program_sequencer_test_app        (Windows: build/Debug/program_sequencer_test_app.exe)
# ============================================================================
cmake_minimum_required(VERSION 3.20)
project(ProgramSequencer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ----------------------------------------------------------------------------
# Static library: the sequencer itself (header-only dependencies on shared/).
# ----------------------------------------------------------------------------
add_library(program_sequencer_lib STATIC
    src/ProgramSequencer.cpp
    # Shared value-type out-of-line definitions (e.g. AxisSet ctor). DataTypes.cpp only depends on
    # DataTypes.h + the standard library, so compiling it here keeps the module self-contained without
    # pulling the whole data_types build tree.
    "${HEXA_SDK_DIR}/data_types/src/DataTypes.cpp"
)

target_include_directories(program_sequencer_lib PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/src"
    # Shared protocol + data-type headers (ProgramDataStruct/ProgramStepStruct, Units, Result).
    "${HEXA_SDK_DIR}/rdt_protocol/src"
    "${HEXA_SDK_DIR}/data_types/src"
)

# ----------------------------------------------------------------------------
# Google Test (FetchContent), guarded so a parent build can provide it.
# ----------------------------------------------------------------------------
if(NOT TARGET GTest::gtest)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

find_package(Threads REQUIRED)

# ----------------------------------------------------------------------------
# Test executable.
# ----------------------------------------------------------------------------
add_executable(program_sequencer_test_app
    tests/gtest_program_sequencer_main.cpp
)

target_link_libraries(program_sequencer_test_app PRIVATE
    program_sequencer_lib
    gtest
    gtest_main
    Threads::Threads
)

enable_testing()
add_test(NAME program_sequencer_tests COMMAND program_sequencer_test_app)

message(STATUS "Configured program_sequencer module. Lib: 'program_sequencer_lib', Test: 'program_sequencer_test_app'")
