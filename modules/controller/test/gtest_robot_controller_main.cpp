#include "gtest/gtest.h"
#include "RobotController.h"
#include "HardwareManager.h"
#include "MotionManager.h"
#include "planner/TrajectoryPlanner.h"
#include "segments/TrajectoryInterpolator.h"
#include "kinematic_solver/KdlKinematicSolver.h"
#include "robot_model/KinematicModel.h"
#include "RobotState.h"
#include "LoggingMacros.h"
#include "RobotConfig.h"

#include <thread>
#include <chrono>
#include <memory>

using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;

// Test Fixture: Sets up the full system stack with Simulation Driver
class RobotControllerIntegrationTest : public ::testing::Test {
protected:
    RobotLimits limits;
    InterfaceConfig hw_config;

    void SetUp() override {
        // 1. Logger (Silence console for cleaner test output, or use Debug to see flow)
        RDT::Logger::Init({std::make_shared<RDT::ConsoleSink>()}, RDT::LogLevel::Info);

        // 2. Robot State
        robot_state = std::make_shared<RobotState>();

        // 3. Kinematics & Limits
        auto model = KinematicModel::createLbrIisy11R1300();
        ASSERT_NE(model, nullptr);

        limits.joint_position_limits_deg.fill({-180.0_deg, 180.0_deg});
        limits.joint_velocity_limits_deg_s.fill(1000.0_deg_s); // High limits for testing

        auto solver = std::make_shared<KdlKinematicSolver>(std::move(model), limits);

        // 4. Hardware (Simulation Mode)
        hw_config.realtime_type = InterfaceConfig::RealtimeInterfaceType::None; // Sim only
        hw_config.simulation_initial_joints.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
        
        auto hw_manager = std::make_shared<HardwareManager>(hw_config, limits);
        // Explicitly init hardware to ensure SimDriver is running
        ASSERT_TRUE(hw_manager->init().isSuccess());

        // 5. Motion Manager (RT)
        // 16ms cycle (increased 4x), 20.0 deg following error threshold (stable simulation tests)
        motion_manager = std::make_shared<MotionManager>(hw_manager, 16, limits, 20.0_deg);
        
        // 6. Planner (NRT)
        auto interpolator = std::make_shared<TrajectoryInterpolator>(solver);
        auto planner = std::make_shared<TrajectoryPlanner>(interpolator, motion_manager);

        // 7. Controller (SUT - System Under Test)
        controller = std::make_unique<RobotController>(
            hw_manager, motion_manager, planner, solver, robot_state
        );
    }

    void TearDown() override {
        // Controller destructor handles cleanup, but we explicitly shutdown Logger
        // to flush buffers.
        controller.reset(); 
        RDT::Logger::Shutdown();
    }

    std::shared_ptr<RobotState> robot_state;
    std::shared_ptr<MotionManager> motion_manager;
    std::unique_ptr<RobotController> controller;
};

// --- Test Cases ---

TEST_F(RobotControllerIntegrationTest, Initialization) {
    // Act
    bool success = controller->initialize();

    // Assert
    ASSERT_TRUE(success);
    EXPECT_EQ(robot_state->getRobotMode(), RobotMode::Idle);
    EXPECT_FALSE(robot_state->hasError());
    EXPECT_FALSE(robot_state->isEStopActive());
}

TEST_F(RobotControllerIntegrationTest, EmergencyStopLifecycle) {
    ASSERT_TRUE(controller->initialize());

    // 1. Simulate E-Stop Request from Network
    NetProtocol::ControlState cmd_estop_on;
    cmd_estop_on.setEStop = true;
    robot_state->processNetworkCommand(cmd_estop_on);

    // Run controller update to process command
    controller->update();

    // Verify E-Stop State
    EXPECT_TRUE(robot_state->isEStopActive());
    EXPECT_EQ(robot_state->getRobotMode(), RobotMode::EStop);

    // 2. Simulate E-Stop Reset Request
    NetProtocol::ControlState cmd_estop_off;
    cmd_estop_off.setEStop = false; // Release button
    cmd_estop_off.resetEStop = true; // Request reset
    robot_state->processNetworkCommand(cmd_estop_off);

    controller->update();

    // E-Stop should be inactive, but mode might still be Error or EStop until Explicit ClearError
    EXPECT_FALSE(robot_state->isEStopActive());

    // 3. Clear Error to return to Idle
    NetProtocol::ControlState cmd_clear;
    cmd_clear.clearError = true;
    robot_state->processNetworkCommand(cmd_clear);

    controller->update();
    EXPECT_EQ(robot_state->getRobotMode(), RobotMode::Idle);
}

TEST_F(RobotControllerIntegrationTest, JoggingExecution) {
    ASSERT_TRUE(controller->initialize());

    // Wait for initial feedback to populate robot_state and warm up the planner
    for(int i=0; i<50; ++i) { // Wait 2 seconds
        controller->update();
        std::this_thread::sleep_for(40ms);
    }

    // Verify initial position is 0
    auto initial_fb = robot_state->getFeedbackTrajectoryPoint();
    EXPECT_NEAR(initial_fb.feedback.joint_actual.GetAt(0).value().get().position.value(), 0.0, 0.01);

    // 1. Send Jog Command: A1 + 10 degrees
    NetProtocol::ControlState cmd;
    cmd.jogRequestId = 1; // Increment ID to trigger processing
    cmd.jogFrame = NetProtocol::JogFrame::JOINT;
    cmd.jogAxis = 0; // A1
    cmd.jogIncrement = 10.0;
    robot_state->processNetworkCommand(cmd);

    // 2. Run simulation loop
    for(int i=0; i<300; ++i) { // Give more time for jogging to finish
        controller->update();
        if (i % 20 == 0) {
             auto fb = robot_state->getFeedbackTrajectoryPoint();
             RDT_LOG_INFO("TEST", "Cycle {}: Pos A1: {}, Queue: {}", i, 
                          fb.feedback.joint_actual.GetAt(0).value().get().position.value(),
                          motion_manager->getCommandQueueSize());
        }
        std::this_thread::sleep_for(40ms);
    }

    // 3. Verify movement
    auto final_fb = robot_state->getFeedbackTrajectoryPoint();
    double pos_a1 = final_fb.feedback.joint_actual.GetAt(0).value().get().position.value();
    
    // Should be close to 10.0.
    EXPECT_GT(pos_a1, 0.1) << "Robot moved too little. Final pos: " << pos_a1; 
}

TEST_F(RobotControllerIntegrationTest, ProgramExecution_MoveJ) {
    ASSERT_TRUE(controller->initialize());

    // 1. Create a simple program
    NetProtocol::ProgramDataStruct prog;
    prog.name = "TestMoveJ";
    
    NetProtocol::ProgramStepStruct step1;
    step1.id = 1;
    step1.type = NetProtocol::StepType::MoveJ;
    // Target: A1 = 20 degrees
    step1.joint_target.SetFromPositionArray({20.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    step1.speed_ratio = 50.0; // 50% speed
    prog.steps.push_back(step1);

    robot_state->updateLoadedProgram(prog);

    // 2. Send RUN command
    NetProtocol::ControlState cmd;
    cmd.programCommand = 1; // 1 = RUN
    robot_state->processNetworkCommand(cmd);

    // 3. Run execution loop
    bool was_running = false;
    bool completed = false;

    for(int i=0; i<400; ++i) { // Up to 16 seconds
        controller->update();
        auto mode = robot_state->getRobotMode();
        
        if (mode == RobotMode::Running) was_running = true;
        
        // If program finished, wait a few more cycles for feedback to catch up
        if (was_running && mode == RobotMode::Idle) {
            // Settling delay: run a few more updates
            for(int j=0; j<10; ++j) {
                std::this_thread::sleep_for(40ms);
                controller->update();
            }

            auto fb = robot_state->getFeedbackTrajectoryPoint();
            double current_pos = fb.feedback.joint_actual.GetAt(0).value().get().position.value();
            if (std::abs(current_pos - 20.0) < 0.2) {
                completed = true;
                break;
            } else {
                // If not reached yet, maybe still settling
                continue;
            }
        }
        
        std::this_thread::sleep_for(40ms);
    }

    EXPECT_TRUE(was_running) << "Robot never entered Running state";
    EXPECT_TRUE(completed) << "Program did not complete or reach target. Actual pos: " 
                           << robot_state->getFeedbackTrajectoryPoint().feedback.joint_actual.GetAt(0).value().get().position.value();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
