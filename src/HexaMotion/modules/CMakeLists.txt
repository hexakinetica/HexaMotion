cmake_minimum_required(VERSION 3.20)
project(HexaMotionTopLevel)

# Add all the modules as subdirectories
add_subdirectory(hardware_hal)
add_subdirectory(motion_manager_rt)
add_subdirectory(kinematics_nrt)
add_subdirectory(planning_nrt)
# Program sequencer: the controller compiles its .cpp directly; this subdirectory contributes the
# module's own 19-test suite (program_sequencer_test_app) to the tree battery (P2 verification).
add_subdirectory(program_sequencer)
