# ==============================================================================
# CMakeLists.txt for HexaCore
# Standalone robot controller core (migrated from RDT-core-old)
# ==============================================================================
cmake_minimum_required(VERSION 3.20)
project(HexaCore LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Dependencies ---
find_package(Threads REQUIRED)

# ==============================================================================
# External Libraries Configuration (KDL, Eigen)
# ==============================================================================

# Определяем пути к библиотекам в зависимости от платформы
if(WIN32)
    # Windows (для тестов)
    set(LIBS_ROOT "C:/libs")
    message(STATUS "Platform: Windows - using libraries from: ${LIBS_ROOT}")
elseif(UNIX)
    # Linux (для деплоя)
    set(LIBS_ROOT "/usr/local")
    # Альтернативные пути для Linux
    if(EXISTS "/opt/libs")
        set(LIBS_ROOT "/opt/libs")
    endif()
    message(STATUS "Platform: Linux - using libraries from: ${LIBS_ROOT}")
else()
    message(WARNING "Unknown platform, trying default paths")
    set(LIBS_ROOT "C:/libs")
endif()

# Прямые пути к библиотекам - проверяем несколько вариантов
set(KDL_INCLUDE_DIR "")
set(KDL_LIB_DIR "")

# Варианты путей для KDL
set(KDL_POSSIBLE_PATHS
    "${LIBS_ROOT}/orocos-kdl/include"      # C:/libs/orocos-kdl/include
    "${LIBS_ROOT}/kdl/include"              # C:/libs/kdl/include
    "${LIBS_ROOT}/orocos-kdl"               # C:/libs/orocos-kdl (если include в корне)
    "${LIBS_ROOT}/kdl"                      # C:/libs/kdl (если include в корне)
)

# Ищем KDL в разных местах
foreach(KDL_PATH ${KDL_POSSIBLE_PATHS})
    if(EXISTS "${KDL_PATH}/kdl/chain.hpp")
        set(KDL_INCLUDE_DIR "${KDL_PATH}")
        message(STATUS "✓ KDL found at: ${KDL_INCLUDE_DIR}")
        break()
    elseif(EXISTS "${KDL_PATH}/chain.hpp")
        # Если chain.hpp прямо в include
        set(KDL_INCLUDE_DIR "${KDL_PATH}")
        message(STATUS "✓ KDL found at: ${KDL_INCLUDE_DIR} (flat structure)")
        break()
    endif()
endforeach()

# Если не нашли, пробуем найти через поиск файла
if(NOT KDL_INCLUDE_DIR)
    # Пробуем найти chain.hpp рекурсивно
    file(GLOB_RECURSE KDL_CHAIN_HPP "${LIBS_ROOT}/*/chain.hpp" "${LIBS_ROOT}/*/kdl/chain.hpp")
    if(KDL_CHAIN_HPP)
        list(GET KDL_CHAIN_HPP 0 FIRST_CHAIN_HPP)
        get_filename_component(KDL_CHAIN_DIR "${FIRST_CHAIN_HPP}" DIRECTORY)
        # Если нашли kdl/chain.hpp, берем родительскую директорию
        if(KDL_CHAIN_DIR MATCHES "kdl$")
            get_filename_component(KDL_INCLUDE_DIR "${KDL_CHAIN_DIR}" DIRECTORY)
        else()
            set(KDL_INCLUDE_DIR "${KDL_CHAIN_DIR}")
        endif()
        message(STATUS "✓ KDL found via search at: ${KDL_INCLUDE_DIR}")
    endif()
endif()

# Определяем директорию с библиотеками
if(KDL_INCLUDE_DIR)
    set(KDL_FOUND TRUE)
    # Пробуем найти lib директорию
    # KDL_INCLUDE_DIR может быть либо include, либо include/kdl, либо корень
    get_filename_component(KDL_BASE_DIR "${KDL_INCLUDE_DIR}" DIRECTORY)
    # Если KDL_INCLUDE_DIR заканчивается на kdl, берем родителя
    if(KDL_INCLUDE_DIR MATCHES "/kdl$")
        get_filename_component(KDL_BASE_DIR "${KDL_INCLUDE_DIR}" DIRECTORY)
    endif()
    
    # Варианты поиска lib директории
    set(KDL_LIB_POSSIBLE_PATHS
        "${KDL_BASE_DIR}/lib"
        "${KDL_BASE_DIR}/lib64"
        "${KDL_BASE_DIR}/bin"  # иногда DLL в bin
        "${LIBS_ROOT}/orocos-kdl/lib"
        "${LIBS_ROOT}/orocos-kdl/lib64"
        "${LIBS_ROOT}/kdl/lib"
    )
    
    foreach(LIB_PATH ${KDL_LIB_POSSIBLE_PATHS})
        if(EXISTS "${LIB_PATH}")
            set(KDL_LIB_DIR "${LIB_PATH}")
            message(STATUS "  KDL lib directory: ${KDL_LIB_DIR}")
            break()
        endif()
    endforeach()
    
    # Если не нашли через пути, ищем файлы библиотек
    if(NOT KDL_LIB_DIR)
        file(GLOB_RECURSE KDL_LIB_FILES 
            "${KDL_BASE_DIR}/*/liborocos-kdl*.a"
            "${KDL_BASE_DIR}/*/orocos-kdl*.lib"
            "${KDL_BASE_DIR}/*/orocos-kdl*.dll"
            "${LIBS_ROOT}/orocos-kdl/*/liborocos-kdl*.a"
            "${LIBS_ROOT}/orocos-kdl/*/orocos-kdl*.lib"
        )
        if(KDL_LIB_FILES)
            list(GET KDL_LIB_FILES 0 FIRST_LIB)
            get_filename_component(KDL_LIB_DIR "${FIRST_LIB}" DIRECTORY)
            message(STATUS "  KDL lib directory (found via search): ${KDL_LIB_DIR}")
        else()
            message(WARNING "  KDL library files not found, will try system paths during linking")
        endif()
    endif()
else()
    set(KDL_FOUND FALSE)
    message(WARNING "⚠ KDL not found. Searched in: ${KDL_POSSIBLE_PATHS}")
endif()

set(EIGEN_INCLUDE_DIR "${LIBS_ROOT}/eigen")

if(EXISTS "${EIGEN_INCLUDE_DIR}/Eigen")
    set(EIGEN_FOUND TRUE)
    message(STATUS "✓ Eigen3 found at: ${EIGEN_INCLUDE_DIR}")
else()
    set(EIGEN_FOUND FALSE)
    message(WARNING "⚠ Eigen3 not found at: ${EIGEN_INCLUDE_DIR}")
endif()

# Установка флагов
if(KDL_FOUND AND EIGEN_FOUND)
    add_definitions(-DUSE_KDL -DUSE_EIGEN -DUSE_REAL_KINEMATICS)
    set(USE_REAL_KINEMATICS ON)
    message(STATUS "✓ Using real kinematics (KDL + Eigen)")
else()
    add_definitions(-DUSE_MOCK_KINEMATICS)
    set(USE_REAL_KINEMATICS OFF)
    message(WARNING "⚠ Using mock kinematics (KDL or Eigen not found)")
endif()

# --- Source Files from RDT-core-old ---
# Note: These should be copied to HexaCore/src/ directories
# For now, we reference them from RDT-core-old

# Controller
set(HEXACORE_CONTROLLER_SOURCES
    src/controller/RobotController.cpp
    src/controller/RobotController.h
)

# Motion Manager
set(HEXACORE_MOTION_MANAGER_SOURCES
    ../RDT-core-old/motion_manager_rt/MotionManager.cpp
    ../RDT-core-old/motion_manager_rt/MotionManager.h
)

# Hardware Interface
set(HEXACORE_HARDWARE_SOURCES
    ../RDT-core-old/motion_interface_hal/MasterHardwareInterface.cpp
    ../RDT-core-old/motion_interface_hal/MasterHardwareInterface.h
    ../RDT-core-old/motion_interface_hal/InternalSimulation.cpp
    ../RDT-core-old/motion_interface_hal/InternalSimulation.h
    ../RDT-core-old/motion_interface_hal/IHardwareInterface.h
    ../RDT-core-old/motion_interface_hal/UdpInterface.cpp
    ../RDT-core-old/motion_interface_hal/UdpInterface.h
    ../RDT-core-old/motion_interface_hal/EthercatInterface.cpp
    ../RDT-core-old/motion_interface_hal/EthercatInterface.h
)

# Planning
set(HEXACORE_PLANNING_SOURCES
    ../RDT-core-old/trajectory_planner_nrt/TrajectoryPlanner.cpp
    ../RDT-core-old/trajectory_planner_nrt/TrajectoryPlanner.h
    ../RDT-core-old/interpolator_nrt/TrajectoryInterpolator.cpp
    ../RDT-core-old/interpolator_nrt/TrajectoryInterpolator.h
    ../RDT-core-old/interpolator_nrt/MotionProfile.cpp
    ../RDT-core-old/interpolator_nrt/MotionProfile.h
)

# Kinematics
set(HEXACORE_KINEMATICS_SOURCES
    ../RDT-core-old/kinematic_solver_nrt/KinematicModel.cpp
    ../RDT-core-old/kinematic_solver_nrt/KinematicModel.h
    ../RDT-core-old/kinematic_solver_nrt/KinematicSolver.h
    ../RDT-core-old/spatial_math_/FrameTransformer.h
)

# KDL solver only if real kinematics is enabled
if(USE_REAL_KINEMATICS)
    list(APPEND HEXACORE_KINEMATICS_SOURCES
        ../RDT-core-old/kinematic_solver_nrt/KdlKinematicSolver.cpp
        ../RDT-core-old/kinematic_solver_nrt/KdlKinematicSolver.h
    )
endif()

# Queue
set(HEXACORE_QUEUE_SOURCES
    ../RDT-core-old/trajectory_queue_lf/TrajectoryQueue.h
)

# --- Executable Definition ---
add_executable(HexaCore
    main.cpp
    ${HEXACORE_CONTROLLER_SOURCES}
    ${HEXACORE_MOTION_MANAGER_SOURCES}
    ${HEXACORE_HARDWARE_SOURCES}
    ${HEXACORE_PLANNING_SOURCES}
    ${HEXACORE_KINEMATICS_SOURCES}
)

# --- Include Directories ---
target_include_directories(HexaCore PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_SOURCE_DIR}/shared"
    # RDT-core-old includes
    "../RDT-core-old/motion_manager_rt"
    "../RDT-core-old/motion_interface_hal"
    "../RDT-core-old/trajectory_planner_nrt"
    "../RDT-core-old/interpolator_nrt"
    "../RDT-core-old/kinematic_solver_nrt"
    "../RDT-core-old/spatial_math_"
    "../RDT-core-old/trajectory_queue_lf"
)

# Добавляем пути к внешним библиотекам
# Eigen нужен даже в mock режиме для некоторых компонентов
if(EIGEN_FOUND)
    target_include_directories(HexaCore PRIVATE "${EIGEN_INCLUDE_DIR}")
endif()

if(USE_REAL_KINEMATICS)
    if(KDL_FOUND)
        target_include_directories(HexaCore PRIVATE "${KDL_INCLUDE_DIR}")
    endif()
endif()

# --- Linking ---
target_link_libraries(HexaCore PRIVATE
    rdt_bridge
    logger
    data_types_rdt
    state_data
    Threads::Threads
)

if(WIN32)
    target_link_libraries(HexaCore PRIVATE ws2_32)
    # MinGW requires libatomic for atomic operations
    target_link_libraries(HexaCore PRIVATE atomic)
endif()

add_executable(JogTest tests/unit/JogTest.cpp ${HEXACORE_CONTROLLER_SOURCES} ${HEXACORE_MOTION_MANAGER_SOURCES} ${HEXACORE_HARDWARE_SOURCES} ${HEXACORE_PLANNING_SOURCES} ${HEXACORE_KINEMATICS_SOURCES})
target_include_directories(JogTest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_SOURCE_DIR}/shared" "../RDT-core-old/motion_manager_rt" "../RDT-core-old/motion_interface_hal" "../RDT-core-old/trajectory_planner_nrt" "../RDT-core-old/interpolator_nrt" "../RDT-core-old/kinematic_solver_nrt" "../RDT-core-old/spatial_math_" "../RDT-core-old/trajectory_queue_lf")
if(EIGEN_FOUND)
    target_include_directories(JogTest PRIVATE "${EIGEN_INCLUDE_DIR}")
endif()
if(USE_REAL_KINEMATICS AND KDL_FOUND)
    target_include_directories(JogTest PRIVATE "${KDL_INCLUDE_DIR}")
endif()
target_link_libraries(JogTest PRIVATE rdt_bridge logger data_types_rdt state_data Threads::Threads)
if(WIN32)
    target_link_libraries(JogTest PRIVATE ws2_32 atomic)
endif()
if(USE_REAL_KINEMATICS AND KDL_FOUND)
    if(WIN32)
        if(EXISTS "${KDL_LIB_DIR}/liborocos-kdl.dll.a")
            target_link_libraries(JogTest PRIVATE "${KDL_LIB_DIR}/liborocos-kdl.dll.a")
        elseif(EXISTS "${KDL_LIB_DIR}/orocos-kdl.lib")
            target_link_libraries(JogTest PRIVATE "${KDL_LIB_DIR}/orocos-kdl.lib")
        elseif(EXISTS "${KDL_LIB_DIR}/liborocos-kdl.a")
            target_link_libraries(JogTest PRIVATE "${KDL_LIB_DIR}/liborocos-kdl.a")
        endif()
    else()
        target_link_libraries(JogTest PRIVATE orocos-kdl)
    endif()
endif()

add_executable(ProgramTest tests/unit/ProgramTest.cpp ${HEXACORE_CONTROLLER_SOURCES} ${HEXACORE_MOTION_MANAGER_SOURCES} ${HEXACORE_HARDWARE_SOURCES} ${HEXACORE_PLANNING_SOURCES} ${HEXACORE_KINEMATICS_SOURCES})
target_include_directories(ProgramTest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_SOURCE_DIR}/shared" "../RDT-core-old/motion_manager_rt" "../RDT-core-old/motion_interface_hal" "../RDT-core-old/trajectory_planner_nrt" "../RDT-core-old/interpolator_nrt" "../RDT-core-old/kinematic_solver_nrt" "../RDT-core-old/spatial_math_" "../RDT-core-old/trajectory_queue_lf")
if(EIGEN_FOUND)
    target_include_directories(ProgramTest PRIVATE "${EIGEN_INCLUDE_DIR}")
endif()
if(USE_REAL_KINEMATICS AND KDL_FOUND)
    target_include_directories(ProgramTest PRIVATE "${KDL_INCLUDE_DIR}")
endif()
target_link_libraries(ProgramTest PRIVATE rdt_bridge logger data_types_rdt state_data Threads::Threads)
if(WIN32)
    target_link_libraries(ProgramTest PRIVATE ws2_32 atomic)
endif()
if(USE_REAL_KINEMATICS AND KDL_FOUND)
    if(WIN32)
        if(EXISTS "${KDL_LIB_DIR}/liborocos-kdl.dll.a")
            target_link_libraries(ProgramTest PRIVATE "${KDL_LIB_DIR}/liborocos-kdl.dll.a")
        elseif(EXISTS "${KDL_LIB_DIR}/orocos-kdl.lib")
            target_link_libraries(ProgramTest PRIVATE "${KDL_LIB_DIR}/orocos-kdl.lib")
        elseif(EXISTS "${KDL_LIB_DIR}/liborocos-kdl.a")
            target_link_libraries(ProgramTest PRIVATE "${KDL_LIB_DIR}/liborocos-kdl.a")
        endif()
    else()
        target_link_libraries(ProgramTest PRIVATE orocos-kdl)
    endif()
endif()

add_executable(JogTestFixed tests/unit/JogTest_Fixed.cpp ${HEXACORE_CONTROLLER_SOURCES} ${HEXACORE_MOTION_MANAGER_SOURCES} ${HEXACORE_HARDWARE_SOURCES} ${HEXACORE_PLANNING_SOURCES} ${HEXACORE_KINEMATICS_SOURCES})
target_include_directories(JogTestFixed PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_SOURCE_DIR}/shared" "../RDT-core-old/motion_manager_rt" "../RDT-core-old/motion_interface_hal" "../RDT-core-old/trajectory_planner_nrt" "../RDT-core-old/interpolator_nrt" "../RDT-core-old/kinematic_solver_nrt" "../RDT-core-old/spatial_math_" "../RDT-core-old/trajectory_queue_lf")
if(EIGEN_FOUND)
    target_include_directories(JogTestFixed PRIVATE "${EIGEN_INCLUDE_DIR}")
endif()
if(USE_REAL_KINEMATICS AND KDL_FOUND)
    target_include_directories(JogTestFixed PRIVATE "${KDL_INCLUDE_DIR}")
endif()
target_link_libraries(JogTestFixed PRIVATE rdt_bridge logger data_types_rdt state_data Threads::Threads)
if(WIN32)
    target_link_libraries(JogTestFixed PRIVATE ws2_32 atomic)
endif()
if(USE_REAL_KINEMATICS AND KDL_FOUND)
    if(WIN32)
        if(EXISTS "${KDL_LIB_DIR}/liborocos-kdl.dll.a")
            target_link_libraries(JogTestFixed PRIVATE "${KDL_LIB_DIR}/liborocos-kdl.dll.a")
        elseif(EXISTS "${KDL_LIB_DIR}/orocos-kdl.lib")
            target_link_libraries(JogTestFixed PRIVATE "${KDL_LIB_DIR}/orocos-kdl.lib")
        elseif(EXISTS "${KDL_LIB_DIR}/liborocos-kdl.a")
            target_link_libraries(JogTestFixed PRIVATE "${KDL_LIB_DIR}/liborocos-kdl.a")
        endif()
    else()
        target_link_libraries(JogTestFixed PRIVATE orocos-kdl)
    endif()
endif()

add_executable(JoggingTest tests/unit/JoggingTest.cpp ${HEXACORE_CONTROLLER_SOURCES} ${HEXACORE_MOTION_MANAGER_SOURCES} ${HEXACORE_HARDWARE_SOURCES} ${HEXACORE_PLANNING_SOURCES} ${HEXACORE_KINEMATICS_SOURCES})
target_include_directories(JoggingTest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_SOURCE_DIR}/shared" "../RDT-core-old/motion_manager_rt" "../RDT-core-old/motion_interface_hal" "../RDT-core-old/trajectory_planner_nrt" "../RDT-core-old/interpolator_nrt" "../RDT-core-old/kinematic_solver_nrt" "../RDT-core-old/spatial_math_" "../RDT-core-old/trajectory_queue_lf")
if(EIGEN_FOUND)
    target_include_directories(JoggingTest PRIVATE "${EIGEN_INCLUDE_DIR}")
endif()
if(USE_REAL_KINEMATICS AND KDL_FOUND)
    target_include_directories(JoggingTest PRIVATE "${KDL_INCLUDE_DIR}")
endif()
target_link_libraries(JoggingTest PRIVATE rdt_bridge logger data_types_rdt state_data Threads::Threads)
if(WIN32)
    target_link_libraries(JoggingTest PRIVATE ws2_32 atomic)
endif()
if(USE_REAL_KINEMATICS AND KDL_FOUND)
    if(WIN32)
        if(EXISTS "${KDL_LIB_DIR}/liborocos-kdl.dll.a")
            target_link_libraries(JoggingTest PRIVATE "${KDL_LIB_DIR}/liborocos-kdl.dll.a")
        elseif(EXISTS "${KDL_LIB_DIR}/orocos-kdl.lib")
            target_link_libraries(JoggingTest PRIVATE "${KDL_LIB_DIR}/orocos-kdl.lib")
        elseif(EXISTS "${KDL_LIB_DIR}/liborocos-kdl.a")
            target_link_libraries(JoggingTest PRIVATE "${KDL_LIB_DIR}/liborocos-kdl.a")
        endif()
    else()
        target_link_libraries(JoggingTest PRIVATE orocos-kdl)
    endif()
endif()

add_executable(ModeVisualizationTest tests/unit/ModeVisualizationTest.cpp ${HEXACORE_CONTROLLER_SOURCES} ${HEXACORE_MOTION_MANAGER_SOURCES} ${HEXACORE_HARDWARE_SOURCES} ${HEXACORE_PLANNING_SOURCES} ${HEXACORE_KINEMATICS_SOURCES})
target_include_directories(ModeVisualizationTest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_SOURCE_DIR}/shared" "../RDT-core-old/motion_manager_rt" "../RDT-core-old/motion_interface_hal" "../RDT-core-old/trajectory_planner_nrt" "../RDT-core-old/interpolator_nrt" "../RDT-core-old/kinematic_solver_nrt" "../RDT-core-old/spatial_math_" "../RDT-core-old/trajectory_queue_lf")
if(EIGEN_FOUND)
    target_include_directories(ModeVisualizationTest PRIVATE "${EIGEN_INCLUDE_DIR}")
endif()
if(USE_REAL_KINEMATICS AND KDL_FOUND)
    target_include_directories(ModeVisualizationTest PRIVATE "${KDL_INCLUDE_DIR}")
endif()
target_link_libraries(ModeVisualizationTest PRIVATE rdt_bridge logger data_types_rdt state_data Threads::Threads)
if(WIN32)
    target_link_libraries(ModeVisualizationTest PRIVATE ws2_32 atomic)
endif()
if(USE_REAL_KINEMATICS AND KDL_FOUND)
    if(WIN32)
        if(EXISTS "${KDL_LIB_DIR}/liborocos-kdl.dll.a")
            target_link_libraries(ModeVisualizationTest PRIVATE "${KDL_LIB_DIR}/liborocos-kdl.dll.a")
        elseif(EXISTS "${KDL_LIB_DIR}/orocos-kdl.lib")
            target_link_libraries(ModeVisualizationTest PRIVATE "${KDL_LIB_DIR}/orocos-kdl.lib")
        elseif(EXISTS "${KDL_LIB_DIR}/liborocos-kdl.a")
            target_link_libraries(ModeVisualizationTest PRIVATE "${KDL_LIB_DIR}/liborocos-kdl.a")
        endif()
    else()
        target_link_libraries(ModeVisualizationTest PRIVATE orocos-kdl)
    endif()
endif()

# Link external libraries if found
if(USE_REAL_KINEMATICS)
    if(KDL_FOUND)
        # Поиск библиотеки KDL
        if(WIN32)
            if(EXISTS "${KDL_LIB_DIR}/liborocos-kdl.dll.a")
                target_link_libraries(HexaCore PRIVATE "${KDL_LIB_DIR}/liborocos-kdl.dll.a")
            elseif(EXISTS "${KDL_LIB_DIR}/orocos-kdl.lib")
                target_link_libraries(HexaCore PRIVATE "${KDL_LIB_DIR}/orocos-kdl.lib")
            elseif(EXISTS "${KDL_LIB_DIR}/liborocos-kdl.a")
                target_link_libraries(HexaCore PRIVATE "${KDL_LIB_DIR}/liborocos-kdl.a")
            else()
                message(WARNING "KDL library file not found in ${KDL_LIB_DIR}")
            endif()
        else()
            # Linux
            if(EXISTS "${KDL_LIB_DIR}/liborocos-kdl.so")
                target_link_libraries(HexaCore PRIVATE "${KDL_LIB_DIR}/liborocos-kdl.so")
            elseif(EXISTS "${KDL_LIB_DIR}/liborocos-kdl.a")
                target_link_libraries(HexaCore PRIVATE "${KDL_LIB_DIR}/liborocos-kdl.a")
            else()
                # Попытка найти через pkg-config или системные пути
                find_library(KDL_LIB orocos-kdl PATHS "${KDL_LIB_DIR}" NO_DEFAULT_PATH)
                if(KDL_LIB)
                    target_link_libraries(HexaCore PRIVATE ${KDL_LIB})
                else()
                    message(WARNING "KDL library not found, trying system paths...")
                    target_link_libraries(HexaCore PRIVATE orocos-kdl)
                endif()
            endif()
        endif()
    endif()
    
    # Eigen - header-only, не требует линковки
    # Но проверим наличие
    if(NOT EIGEN_FOUND)
        message(WARNING "Eigen3 not found - some features may not work")
    endif()
endif()

message(STATUS "")
message(STATUS "========================================")
message(STATUS "HexaCore Configuration Summary")
message(STATUS "========================================")
message(STATUS "Platform: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Libraries root: ${LIBS_ROOT}")
if(USE_REAL_KINEMATICS)
    message(STATUS "Kinematics: REAL (KDL + Eigen)")
    if(KDL_FOUND)
        message(STATUS "  KDL include: ${KDL_INCLUDE_DIR}")
        message(STATUS "  KDL lib: ${KDL_LIB_DIR}")
    endif()
    if(EIGEN_FOUND)
        message(STATUS "  Eigen include: ${EIGEN_INCLUDE_DIR}")
    endif()
else()
    message(STATUS "Kinematics: MOCK (for testing)")
endif()
message(STATUS "Controller: migrated from RDT-core-old")
message(STATUS "State: using RobotState from shared/")
message(STATUS "Network: using RdtServer from shared/")
message(STATUS "========================================")
