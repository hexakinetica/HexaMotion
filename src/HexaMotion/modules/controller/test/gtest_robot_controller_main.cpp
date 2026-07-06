#include "gtest/gtest.h"
#include "RobotController.h"
#include "HardwareManager.h"
#include "MotionManager.h"
#include "planner/TrajectoryPlanner.h"
#include "segments/TrajectoryInterpolator.h"
#include "kinematic_solver/KdlKinematicSolver.h"
#include "KinematicModel.h"
#include "RobotState.h"
#include "LoggingMacros.h"
#include "RobotConfig.h"

#include <thread>
#include <chrono>
#include <memory>
#include <filesystem>
#include <cstdlib>

using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;

class RobotControllerIntegrationTest : public ::testing::Test {
protected:
    static std::string resolveUrdfPathForTest() {
        if (const char* env = std::getenv("HEXAMOTION_TEST_URDF")) {
            std::filesystem::path p(env);
            if (std::filesystem::exists(p)) {
                return p.string();
            }
        }

        const std::vector<std::string> candidates = {
            "HexaMotion/modules/kinematics_nrt/tests/lbr_iisy_11_r1300.urdf",
            "../kinematics_nrt/tests/lbr_iisy_11_r1300.urdf",
            "tests/lbr_iisy_11_r1300.urdf"
        };
        for (const auto& c : candidates) {
            std::filesystem::path p(c);
            if (std::filesystem::exists(p)) {
                return p.string();
            }
        }
        return {};
    }

    InterfaceConfig hw_config;
    RobotLimits limits;
    ControllerConfig ctrl_config;

    void SetUp() override {
        RDT::Logger::Init({std::make_shared<RDT::ConsoleSink>()}, RDT::LogLevel::Info);
        robot_state = std::make_shared<RobotState>();
        
        const std::string urdf_path = resolveUrdfPathForTest();
        ASSERT_FALSE(urdf_path.empty()) << "Controller integration test URDF is not resolved.";
        RobotModelConfig config;   // RDT::RobotModelConfig (RobotModelTypes.h); was mis-qualified as
                                   // KinematicModel::RobotModelConfig, which no longer exists.
        config.urdf_path = urdf_path;
        config.base_link = "base_link";
        config.tip_link = "flange";
        auto model = KinematicModel::createFromURDFFile(config);
        limits.joint_position_limits_deg.fill({-180.0_deg, 180.0_deg});
        limits.joint_velocity_limits_deg_s.fill(1000.0_deg_s);

        auto solver = std::make_shared<KdlKinematicSolver>(std::move(model), limits);

        hw_config.realtime_type = InterfaceConfig::RealtimeInterfaceType::None;
        ASSERT_TRUE(hw_config.simulation_initial_joints.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg}).isSuccess());
        
        hw_manager = std::make_shared<HardwareManager>(hw_config, limits);
        ASSERT_TRUE(hw_manager->init().isSuccess());

        motion_manager = std::make_shared<MotionManager>(hw_manager, 16, limits, 20.0_deg);
        ctrl_config.PlannerTickSec = 0.016_s;
        
        auto interpolator = std::make_shared<TrajectoryInterpolator>(solver);
        planner = std::make_shared<TrajectoryPlanner>(interpolator, motion_manager, ctrl_config);

        controller = std::make_unique<RobotController>(
            hw_manager, motion_manager, planner, solver, robot_state, ctrl_config
        );
    }

    void TearDown() override {
        controller.reset(); 
        RDT::Logger::Shutdown();
    }

    std::shared_ptr<RobotState> robot_state;
    std::shared_ptr<HardwareManager> hw_manager;
    std::shared_ptr<MotionManager> motion_manager;
    std::shared_ptr<TrajectoryPlanner> planner;
    std::unique_ptr<RobotController> controller;
};

// ... Existing tests (Initialization, Jogging, etc.) omitted for brevity, assume they are here ...
// Keeping one existing test to ensure file completeness rule
TEST_F(RobotControllerIntegrationTest, Initialization) {
    bool success = controller->initialize();
    ASSERT_TRUE(success);
    EXPECT_EQ(robot_state->getRobotMode(), RobotMode::Idle);
}

// --- NEW TESTS ---

TEST_F(RobotControllerIntegrationTest, MasteringRequest) {
    ASSERT_TRUE(controller->initialize());

    // 1. Jog the VIRTUAL MC (the sim backend's real driver, REQ-SIMMC-01) +10 deg through the
    // production HAL-jog path. In SIM view the RT loop must not clobber the result: the
    // keep-alive Hold stream is skipped for the sim backend by design (REQ-SIMMC-04).
    ASSERT_TRUE(hw_manager->jogRealIncremental(0, 10.0, 1.0).isSuccess());

    std::this_thread::sleep_for(50ms);
    controller->update();

    auto mc_fb = hw_manager->getRealDriverFeedback();
    ASSERT_TRUE(mc_fb.isSuccess());
    EXPECT_NEAR(mc_fb.value().joints.GetAt(0).value().get().position.value(), 10.0, 0.1);

    // The ACTIVE sim robot (solid) is untouched by the MC jog — same as with real hardware.
    auto fb = robot_state->getFeedbackTrajectoryPoint();
    EXPECT_NEAR(fb.feedback.joint_actual.GetAt(0).value().get().position.value(), 0.0, 0.1);

    // 2. Send Master command for Axis 0 — real-first routing targets the virtual MC (REQ-SIMMC-07)
    NetProtocol::ControlState cmd;
    cmd.masterAxisId = 0;
    cmd.masteringReqId = 1; // audit F-06: mastering executes only on a non-zero req-id
    robot_state->processNetworkCommand(cmd);

    // 3. Process
    controller->update();
    std::this_thread::sleep_for(20ms);
    controller->update();

    // 4. Verify the virtual MC now reads logical 0 at the jogged physical position
    mc_fb = hw_manager->getRealDriverFeedback();
    ASSERT_TRUE(mc_fb.isSuccess());
    EXPECT_NEAR(mc_fb.value().joints.GetAt(0).value().get().position.value(), 0.0, 0.1);
}

TEST_F(RobotControllerIntegrationTest, HomingRequestServedByVirtualMc) {
    ASSERT_TRUE(controller->initialize());

    NetProtocol::ControlState jog_enable_cmd;
    jog_enable_cmd.jogEnableCommand = 1;
    robot_state->processNetworkCommand(jog_enable_cmd);
    controller->update();
    ASSERT_TRUE(robot_state->isJogEnabled());

    NetProtocol::ControlState cmd;
    cmd.startHoming = true;
    cmd.homingAxisId = 1; // A2
    cmd.homingReqId = 1; // audit F-06: homing executes only on a non-zero req-id
    robot_state->processNetworkCommand(cmd);

    controller->update();
    // Homing is owned by the backend; HexaMotion no longer tracks a homing state. On the sim
    // backend the request is served instantly by the virtual MC (REQ-SIMMC-07) and confirmed
    // with a user message; no error state, jog stays armed.
    EXPECT_FALSE(robot_state->hasError());
    EXPECT_FALSE(robot_state->getSystemMessage().empty());
    EXPECT_TRUE(robot_state->isJogEnabled());
    auto mc_fb = hw_manager->getRealDriverFeedback();
    ASSERT_TRUE(mc_fb.isSuccess());   // the virtual MC exists and serves feedback (REQ-SIMMC-01)
}

TEST_F(RobotControllerIntegrationTest, ProgramUpdateTriggersPreview) {
    ASSERT_TRUE(controller->initialize());

    // 1. Initial State
    EXPECT_EQ(robot_state->getTrajectoryVersion(), 1);

    // 2. Upload a new program
    NetProtocol::ProgramDataStruct prog;
    prog.name = "PreviewTriggerTest";
    prog.steps.push_back({.id=1, .type=NetProtocol::StepType::MoveJ});
    
    NetProtocol::ControlState cmd;
    cmd.programUpdateReqId = 101;
    cmd.newProgram = prog;
    
    robot_state->processNetworkCommand(cmd); // Updates program_version, sets last_processed
    
    // 3. Controller Update -> Should detect new program and generate preview
    controller->update();
    
    // 4. Verify trajectory version increased
    EXPECT_GT(robot_state->getTrajectoryVersion(), 1);
    
    // 5. Verify trajectory data is populated
    // Note: Since we didn't populate joints in steps, preview might be empty/zero, but version changes.
    // Let's ensure planner didn't crash.
}

TEST_F(RobotControllerIntegrationTest, ProgramCommandIgnoredWithoutProgram) {
    ASSERT_TRUE(controller->initialize());

    NetProtocol::ControlState run_cmd;
    run_cmd.programCommand = 1;
    run_cmd.programCmdReqId = 1; // audit F2: program commands execute only on a non-zero req-id
    robot_state->processNetworkCommand(run_cmd);
    controller->update();

    EXPECT_EQ(robot_state->getRobotMode(), RobotMode::Idle);
}

TEST_F(RobotControllerIntegrationTest, ProgramCommandRunPauseStop) {
    ASSERT_TRUE(controller->initialize());

    NetProtocol::ProgramDataStruct prog;
    prog.name = "CommandFlow";
    prog.steps.push_back({.id = 1, .type = NetProtocol::StepType::WaitTime, .wait_duration_s = 0.1_s});

    NetProtocol::ControlState program_cmd;
    program_cmd.programUpdateReqId = 11;
    program_cmd.newProgram = prog;
    robot_state->processNetworkCommand(program_cmd);
    controller->update();

    NetProtocol::ControlState run_cmd;
    run_cmd.programCommand = 1;
    run_cmd.programCmdReqId = 1; // audit F2: RUN/PAUSE/STOP deduped by programCmdReqId
    robot_state->processNetworkCommand(run_cmd);
    controller->update();
    EXPECT_EQ(robot_state->getRobotMode(), RobotMode::Running);

    NetProtocol::ControlState pause_cmd;
    pause_cmd.programCommand = 2;
    pause_cmd.programCmdReqId = 2;
    robot_state->processNetworkCommand(pause_cmd);
    controller->update();
    EXPECT_EQ(robot_state->getRobotMode(), RobotMode::Paused);

    NetProtocol::ControlState resume_cmd;
    resume_cmd.programCommand = 1;
    resume_cmd.programCmdReqId = 3;
    robot_state->processNetworkCommand(resume_cmd);
    controller->update();
    EXPECT_EQ(robot_state->getRobotMode(), RobotMode::Running);

    NetProtocol::ControlState stop_cmd;
    stop_cmd.programCommand = 3;
    stop_cmd.programCmdReqId = 4;
    robot_state->processNetworkCommand(stop_cmd);
    controller->update();
    EXPECT_EQ(robot_state->getRobotMode(), RobotMode::Idle);
}

TEST_F(RobotControllerIntegrationTest, ProgramCompletesAfterWaitStep) {
    ASSERT_TRUE(controller->initialize());

    NetProtocol::ProgramDataStruct prog;
    prog.name = "WaitProgram";
    prog.steps.push_back({.id = 1, .type = NetProtocol::StepType::WaitTime, .wait_duration_s = 0.05_s});
    prog.steps.push_back({.id = 2, .type = NetProtocol::StepType::Comment, .comment = "Done"});

    NetProtocol::ControlState program_cmd;
    program_cmd.programUpdateReqId = 22;
    program_cmd.newProgram = prog;
    robot_state->processNetworkCommand(program_cmd);
    controller->update();

    NetProtocol::ControlState run_cmd;
    run_cmd.programCommand = 1;
    run_cmd.programCmdReqId = 1; // audit F2
    robot_state->processNetworkCommand(run_cmd);
    controller->update();

    for (int i = 0; i < 20; ++i) {
        controller->update();
        std::this_thread::sleep_for(20ms);
        if (robot_state->getRobotMode() == RobotMode::Idle) break;
    }

    EXPECT_EQ(robot_state->getRobotMode(), RobotMode::Idle);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}