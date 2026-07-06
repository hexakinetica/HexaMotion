@echo off
REM ==============================================================================
REM Build and Run script for Kinematics NRT Module Test
REM Uses MinGW and CMake.
REM ==============================================================================

setlocal enabledelayedexpansion

REM --- Setup Paths (Based on system detection) ---
set MINGW_PATH=C:\Qt\Tools\mingw1310_64\bin
set CMAKE_PATH=C:\Qt\Tools\CMake_64\bin

REM --- CMake Search Paths ---
REM Points to orocos-kdl and eigen installation directories.
REM GTest is fetched via FetchContent in CMakeLists.txt.
set CMAKE_PREFIX_PATH=C:\libs\orocos-kdl;C:\libs\eigen

REM --- Add to PATH ---
REM Add KDL bin path for runtime DLLs
set KDL_BIN_PATH=C:\libs\orocos-kdl\bin
set PATH=%MINGW_PATH%;%CMAKE_PATH%;%KDL_BIN_PATH%;%PATH%

REM --- Check if tools exist ---
where gcc.exe >nul 2>&1
if errorlevel 1 (
    echo ERROR: gcc.exe not found. Check MINGW_PATH: %MINGW_PATH%
    exit /b 1
)

where cmake.exe >nul 2>&1
if errorlevel 1 (
    echo ERROR: cmake.exe not found. Check CMAKE_PATH: %CMAKE_PATH%
    exit /b 1
)

echo ==============================================================================
echo Building Kinematics NRT Module Test
echo ==============================================================================
echo.

REM --- Create build directory ---
if not exist "build" mkdir build
cd build

REM --- Configure with MinGW ---
echo [1/3] Configuring CMake with MinGW...
cmake .. -G "MinGW Makefiles" ^
    -DCMAKE_C_COMPILER="%MINGW_PATH%\gcc.exe" ^
    -DCMAKE_CXX_COMPILER="%MINGW_PATH%\g++.exe" ^
    -DCMAKE_BUILD_TYPE=Debug ^
    -DCMAKE_PREFIX_PATH="%CMAKE_PREFIX_PATH%"

if errorlevel 1 (
    echo ERROR: CMake configuration failed!
    cd ..
    exit /b 1
)

REM --- Build ---
echo.
echo [2/3] Building Kinematics NRT Test...
cmake --build . --config Debug -j %NUMBER_OF_PROCESSORS%

if errorlevel 1 (
    echo ERROR: Build failed!
    cd ..
    exit /b 1
)

REM --- Summary ---
echo.
echo [3/3] Build complete!
echo.
echo Test executable: build\kinematics_nrt_test_app.exe
echo.

REM --- Run tests ---
echo Starting Kinematics NRT GTests...
echo.
kinematics_nrt_test_app.exe

cd ..
