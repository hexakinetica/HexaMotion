@echo off
REM ==============================================================================
REM Build and Run script for HexaCore
REM Uses MinGW and CMake (same as build_all.bat)
REM ==============================================================================

setlocal enabledelayedexpansion

REM --- Setup Paths ---
set MINGW_PATH=C:\Qt\Tools\mingw1310_64\bin
set CMAKE_PATH=C:\Qt\Tools\CMake_64\bin

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
echo Building HexaCore Robot Controller
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
    -DCMAKE_BUILD_TYPE=Release

if errorlevel 1 (
    echo ERROR: CMake configuration failed!
    cd ..
    exit /b 1
)

REM --- Build ---
echo.
echo [2/3] Building HexaCore...
cmake --build . --config Release -j %NUMBER_OF_PROCESSORS%

if errorlevel 1 (
    echo ERROR: Build failed!
    cd ..
    exit /b 1
)

REM --- Summary ---
echo.
echo [3/3] Build complete!
echo.
echo Executable is in: build\bin\HexaCore.exe
echo.

REM --- Ask to run ---
set /p RUN_NOW="Do you want to run HexaCore now? (Y/N): "
if /i "%RUN_NOW%"=="Y" (
    echo.
    echo Starting HexaCore...
    echo Press Ctrl+C to stop
    echo.
    cd bin
    HexaCore.exe
    cd ..
) else (
    echo.
    echo To run manually:
    echo   build\bin\HexaCore.exe
    echo.
)

cd ..
pause

