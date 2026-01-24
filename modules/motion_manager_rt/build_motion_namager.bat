@echo off
setlocal

REM Create a build directory if it doesn't exist
if not exist build (
    mkdir build
)

REM Change to the build directory
cd build

REM Configure the project using CMake with MinGW Makefiles generator
cmake .. -G "MinGW Makefiles"

REM Check if CMake configuration was successful
if %errorlevel% neq 0 (
    echo CMake configuration failed.
    exit /b %errorlevel%
)

REM Build the project
cmake --build .

REM Check if the build was successful
if %errorlevel% neq 0 (
    echo Build failed.
    exit /b %errorlevel%
)

REM Run the tests
echo Running tests...
motion_manager_test_app.exe

endlocal
