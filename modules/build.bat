@echo off
REM Create a build directory if it doesn't exist
if not exist build (
    mkdir build
)

REM Navigate to the build directory
cd build

REM Configure the project using CMake and the MinGW Makefiles generator
cmake .. -G "MinGW Makefiles"

REM Build the project
cmake --build .

REM Check for build errors
if %errorlevel% neq 0 (
    echo Build failed.
    exit /b %errorlevel%
)

echo Build successful.
