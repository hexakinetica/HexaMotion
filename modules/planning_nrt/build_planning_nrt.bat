@echo off
setlocal

REM --- Move to module directory if not already there ---
cd /d "%~dp0"

echo --- Configuring the planning_nrt module with CMake ---
REM Create a build directory if it doesn't exist
if not exist build (
    mkdir build
)

REM Configure the project using CMake and the MinGW Makefiles generator
REM We use the module's own directory as source
cmake -B build -G "MinGW Makefiles"
if %errorlevel% neq 0 (
    echo CMake configuration failed.
    exit /b %errorlevel%
)

echo.
echo --- Building the planning_nrt_test_app ---
REM Build the test app target
cmake --build build --target planning_nrt_test_app
if %errorlevel% neq 0 (
    echo Build failed.
    exit /b %errorlevel%
)

echo.
echo --- Running Google Tests ---
REM The executable path is relative to the build directory
set TEST_EXECUTABLE=build\planning_nrt_test_app.exe

if not exist %TEST_EXECUTABLE% (
    echo Test executable not found at: %TEST_EXECUTABLE%
    exit /b 1
)

%TEST_EXECUTABLE%

endlocal

