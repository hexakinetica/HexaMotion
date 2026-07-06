@echo off
REM ==============================================================================
REM Build and Run script for Hardware HAL Module Test
REM Uses MinGW and CMake.
REM Assumes GTest and Shared Libraries are findable by CMake.
REM ==============================================================================

setlocal enabledelayedexpansion

REM --- Setup Paths (Adjust these to your system) ---
set MINGW_PATH=C:\Qt\Tools\mingw1310_64\bin
set CMAKE_PATH=C:\Qt\Tools\CMake_64\bin
set CMAKE_PREFIX_PATH="C:\libs\gtest;C:\path\to\HexaMotion\shared_build"

REM --- Add to PATH ---
set PATH=%MINGW_PATH%;%CMAKE_PATH%;%PATH%

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
echo Building Hardware HAL Module Test
echo ==============================================================================
echo.

REM --- Create build directory ---
if not exist "build" mkdir build
cd build

REM --- Configure with MinGW ---
echo [1/3] Configuring CMake with MinGW...
cmake .. -G "MinGW Makefiles" ^
    -DCMAKE_C_COMPILER=%MINGW_PATH%\gcc.exe ^
    -DCMAKE_CXX_COMPILER=%MINGW_PATH%\g++.exe ^
    -DCMAKE_BUILD_TYPE=Debug ^
    -DCMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH%

if errorlevel 1 (
    echo ERROR: CMake configuration failed!
    cd ..
    exit /b 1
)

REM --- Build ---
echo.
echo [2/3] Building Hardware HAL Test...
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
echo Test executable is in: build\hardware_hal_test_app.exe
echo.

REM --- Run tests automatically ---
echo.
echo Starting Hardware HAL GTests...
echo Press Ctrl+C to stop the Python emulator if it doesn't close automatically.
echo.

REM Ensure the python script is findable
set PYTHONPATH=..;%PYTHONPATH%

echo ==============================================================================
echo [1/4] Running Sim Tests...
echo ==============================================================================
test_hal_sim.exe
if errorlevel 1 goto :test_fail

echo.
echo ==============================================================================
echo [2/4] Running Governor Tests...
echo ==============================================================================
test_hal_governor.exe
if errorlevel 1 goto :test_fail

echo.
echo ==============================================================================
echo [3/4] Running Safety Tests...
echo ==============================================================================
test_hal_safety.exe
if errorlevel 1 goto :test_fail

echo.
echo ==============================================================================
echo [4/4] Running Realtime Tests...
echo ==============================================================================
test_hal_realtime.exe
if errorlevel 1 goto :test_fail

echo.
echo ==============================================================================
echo ALL TESTS PASSED!
echo ==============================================================================
goto :eof

:test_fail
echo.
echo ==============================================================================
echo SOME TESTS FAILED!
echo ==============================================================================
exit /b 1

cd ..
pause
