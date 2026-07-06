# ==============================================================================
# HexaMotion - real-time robot controller (HexaCore) application repository.
# ------------------------------------------------------------------------------
# Independent, production-ready repository. It links directly against the single
# HexaMotion-SDK foundation repository - there is NO local copy of the SDK.
#
# HexaMotion-SDK is resolved as:
#   1) a git submodule at external/HexaMotion-SDK (published layout), or
#   2) the sibling HexaMotion-SDK repository in the HexaKinetica dev workspace.
# Either way it is the ONE source of the foundation code, third_party and robots.
# ==============================================================================
cmake_minimum_required(VERSION 3.20)
project(HexaMotion LANGUAGES CXX)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # The mono-repo build set this; keep it so the module sources compile identically.
    add_compile_options(-Wno-unused-result)
endif()

# --- Resolve HexaMotion-SDK (submodule first, sibling fallback) -------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/HexaMotion-SDK/CMakeLists.txt")
    get_filename_component(HEXA_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/HexaMotion-SDK" ABSOLUTE)
else()
    get_filename_component(HEXA_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../HexaMotion-SDK" ABSOLUTE)
endif()
if(NOT EXISTS "${HEXA_SDK_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "HexaMotion-SDK not found at '${HEXA_SDK_DIR}'. Add it as a submodule (external/HexaMotion-SDK) or place it beside this repository.")
endif()
set(HEXA_SDK_DIR "${HEXA_SDK_DIR}" CACHE PATH "Path to the HexaMotion-SDK foundation repository" FORCE)
message(STATUS "HexaMotion: using HexaMotion-SDK at ${HEXA_SDK_DIR}")

# Build the SDK umbrella (foundation targets + vendored third_party + GoogleTest). It publishes
# HEXA_THIRD_PARTY_DIR as a CACHE variable, so the RT modules resolve Eigen/KDL/gtest with no edits.
add_subdirectory("${HEXA_SDK_DIR}" "${CMAKE_BINARY_DIR}/HexaMotion-SDK")

# --- Motion modules + the HexaCore controller executable --------------------------------------------
# Their CMake files reference the SDK sources/includes through ${HEXA_SDK_DIR} (set above).
add_subdirectory(src/HexaMotion/modules)
add_subdirectory(src/HexaMotion)

message(STATUS "Configured HexaMotion (HexaCore RT controller) against HexaMotion-SDK.")
