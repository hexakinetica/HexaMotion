cmake_minimum_required(VERSION 3.20)
project(HexaMotionTopLevel)

# Add all the modules as subdirectories
add_subdirectory(hardware_hal)
add_subdirectory(motion_manager_rt)
add_subdirectory(kinematics_nrt)
add_subdirectory(planning_nrt)
