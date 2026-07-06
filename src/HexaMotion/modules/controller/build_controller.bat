@echo off
setlocal enabledelayedexpansion

REM Setup Paths (Modify as needed)
set MINGW_PATH=C:\Qt\Tools\mingw1310_64\bin
set CMAKE_PATH=C:\Qt\Tools\CMake_64\bin
set CMAKE_PREFIX_PATH="C:\libs\gtest;C:\libs\orocos-kdl;C:\libs\eigen;C:\path\to\shared"

set PATH=%MINGW_PATH%;%CMAKE_PATH%;%PATH%

if not exist "build" mkdir build
cd build

echo Configuring Controller...
cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH%
if errorlevel 1 exit /b 1

echo Building...
cmake --build . -j %NUMBER_OF_PROCESSORS%
if errorlevel 1 exit /b 1

echo Running Tests...
controller_test_app.exe

cd ..