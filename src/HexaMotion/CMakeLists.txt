# ============================================================================
# CMakeLists.txt for HexaMotion (HexaCore entry point)
# ============================================================================
cmake_minimum_required(VERSION 3.20)
project(HexaCore LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

if(WIN32)
    target_link_libraries(network_driver PRIVATE ws2_32)
endif()

# ----------------------------------------------------------------------------
# hexacore_runtime — the ONE controller assembly (ControllerRuntime), shared by
# HexaCore.exe (networked edition) and HexaStudioDesktop.exe (single-process
# edition). Compiles the controller + sequencer sources exactly once for both.
# ----------------------------------------------------------------------------
add_library(hexacore_runtime STATIC
    "${CMAKE_CURRENT_LIST_DIR}/app/ControllerRuntime.h"
    "${CMAKE_CURRENT_LIST_DIR}/app/ControllerRuntime.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/modules/controller/src/RobotController.cpp"
    # Program sequencer: the flat-program interpreter driven by RobotController (loops/branches/waits).
    "${CMAKE_CURRENT_LIST_DIR}/modules/program_sequencer/src/ProgramSequencer.cpp"
)

target_include_directories(hexacore_runtime PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/app"
    "${HEXA_SDK_DIR}/data_types/src"
    "${HEXA_SDK_DIR}/logger/src"
    "${HEXA_SDK_DIR}/rdt_protocol/src"
    "${HEXA_SDK_DIR}/state_data/src"
    "${HEXA_SDK_DIR}/persistence/src"
    "${HEXA_SDK_DIR}/rdt_bridge/src"
    "${CMAKE_CURRENT_LIST_DIR}/modules/controller/src"
    "${CMAKE_CURRENT_LIST_DIR}/modules/program_sequencer/src"
    "${CMAKE_CURRENT_LIST_DIR}/modules/hardware_hal/src"
    "${CMAKE_CURRENT_LIST_DIR}/modules/motion_manager_rt/src"
    "${CMAKE_CURRENT_LIST_DIR}/modules/trajectory_queue_lf/src"
    "${CMAKE_CURRENT_LIST_DIR}/modules/planning_nrt/src"
    "${CMAKE_CURRENT_LIST_DIR}/modules/kinematics_nrt/src"
)

target_compile_features(hexacore_runtime PUBLIC cxx_std_20)

target_link_libraries(hexacore_runtime
    PUBLIC
        data_types
        logger
        rdt_protocol
        StateData
        persistence
        rdt_bridge
        network_driver
        hardware_hal_lib
        motion_manager_lib
        planning_nrt_lib
        kinematics_nrt_lib
        robot_model
        Threads::Threads
)

if(WIN32)
    target_link_libraries(hexacore_runtime PUBLIC ws2_32)
endif()

# ----------------------------------------------------------------------------
# HexaCore — the networked-edition console executable (thin composition root).
# ----------------------------------------------------------------------------
add_executable(HexaCore
    main.cpp
)

target_link_libraries(HexaCore
    PRIVATE
        hexacore_runtime
)

# ============================================================================
# Test: controller_test_app (data-pipeline audit)
# ----------------------------------------------------------------------------
# The controller module's own CMakeLists builds this test only in a standalone module build, which
# on this machine is blocked by an incomplete vendored orocos-kdl. The main tree builds KDL from
# source and already provides every dependency + the vendored GTest, so we register the same test
# here (additive; does not touch HexaCore). Run with HEXAMOTION_TEST_URDF pointing at the model, e.g.
#   modules/kinematics_nrt/tests/lbr_iisy_11_r1300.urdf
# ============================================================================
if(TARGET GTest::gtest)
    # The test links hexacore_runtime: the SAME RobotController/ProgramSequencer objects the
    # products ship (they were compiled here a second time before the runtime lib existed).
    # hexacore_runtime's PUBLIC includes/links provide the whole dependency surface.
    add_executable(controller_test_app
        "${CMAKE_CURRENT_LIST_DIR}/modules/controller/test/gtest_robot_controller_main.cpp"
    )
    target_link_libraries(controller_test_app
        PRIVATE
            hexacore_runtime
            GTest::gtest
            GTest::gtest_main
    )
endif()
