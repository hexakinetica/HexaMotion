#include "gtest/gtest.h"
#include "MotionManager.h"
#include "HardwareManager.h"
#include "RobotConfig.h"
#include "Logger.h"
#include "LoggingMacros.h"
#include "Units.h"

#include <thread>
#include <chrono>
#include <memory>

using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;

bool waitForState(const std::unique_ptr<MotionManager>& mm, RTState target_state, std::chrono::milliseconds timeout) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
        if (mm->getCurrentState() == target_state) return true;
        std::this_thread::sleep_for(5ms);
    }
    return false;
}

// Helper to drain the feedback queue for a certain duration
void drainFeedbackQueue(MotionManager& mm, std::chrono::milliseconds duration) {
    auto start_time = std::chrono::steady_clock::now();
    TrajectoryPoint dummy;
    while (std::chrono::steady_clock::now() - start_time < duration) {
        while (mm.dequeueFeedback(dummy)) {}
        std::this_thread::sleep_for(10ms);
    }
}

class MotionManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Use a minimal config with simulation only
        config.realtime_type = InterfaceConfig::RealtimeInterfaceType::None;
        config.simulation_initial_joints.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});

        // Define limits for tests
        limits.joint_velocity_limits_deg_s.fill(1000.0_deg_s); // Fast limits, we are not testing governor here
        limits.joint_position_limits_deg.fill({-180.0_deg, 180.0_deg});
        limits.joint_position_limits_deg[0] = {-90.0_deg, 90.0_deg}; // Tight limit on A1 for testing

        // Create and initialize HardwareManager
        hw_manager = std::make_shared<HardwareManager>(config, limits);
        ASSERT_TRUE(hw_manager->init().isSuccess());
    }

    void TearDown() override {
        // Clean shutdown
        if (mm) {
            mm->stop();
        }
        if (hw_manager) {
            hw_manager->shutdown();
        }
    }

    InterfaceConfig config;
    RobotLimits limits;
    std::shared_ptr<HardwareManager> hw_manager;
    std::unique_ptr<MotionManager> mm;
};

// Test basic initialization, start, and stop
TEST_F(MotionManagerTest, InitStartStop) {
    mm = std::make_unique<MotionManager>(hw_manager, 100, limits, 5.0_deg);
    EXPECT_EQ(mm->getCurrentState(), RTState::Idle);
    EXPECT_TRUE(mm->start());
    std::this_thread::sleep_for(200ms); // Let the thread run a few cycles (at least 2 ticks)
    EXPECT_TRUE(mm->start()); // Calling start again should be safe
    mm->stop();
}

// Test that a simple, valid point is executed correctly
TEST_F(MotionManagerTest, ExecutesValidMotion) {
    mm = std::make_unique<MotionManager>(hw_manager, 100, limits, 5.0_deg);
    ASSERT_TRUE(mm->start());
    
    TrajectoryPoint target;
    // Enqueue multiple points to ensure the state stays in "Moving" long enough for the test to observe it
    for (int i = 1; i <= 10; ++i) {
        target.command.joint_target.SetFromPositionArray({1.0_deg * i, 2.0_deg * i, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
        ASSERT_TRUE(mm->enqueueCommand(target));
    }
    
    // Wait for the motion to start and complete
    EXPECT_TRUE(waitForState(mm, RTState::Moving, 500ms));
    EXPECT_TRUE(waitForState(mm, RTState::Idle, 2000ms)); // 10 points * 100ms = 1s, so 2s is safe

    // Check the final feedback
    TrajectoryPoint fb;
    bool received = false;
    for(int i = 0; i < 50; ++i) { // Try to get the last feedback point
        if (mm->dequeueFeedback(fb)) {
            received = true;
        }
    }
    ASSERT_TRUE(received);
    EXPECT_NEAR(fb.feedback.joint_actual.GetAt(0).value().get().position.value(), 10.0, 0.01);
    EXPECT_NEAR(fb.feedback.joint_actual.GetAt(1).value().get().position.value(), 20.0, 0.01);
    EXPECT_EQ(fb.feedback.rt_state, RTState::Idle);
}

// Test Position Limit validation (REQ from refactoring plan)
TEST_F(MotionManagerTest, RejectsPositionLimitViolation) {
    mm = std::make_unique<MotionManager>(hw_manager, 100, limits, 5.0_deg);
    ASSERT_TRUE(mm->start());

    TrajectoryPoint illegal_target;
    // A1 limit is [-90, 90]
    illegal_target.command.joint_target.SetFromPositionArray({100.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});

    ASSERT_TRUE(mm->enqueueCommand(illegal_target));

    // MM should enter error state
    EXPECT_TRUE(waitForState(mm, RTState::Error, 500ms));
    
    // Check feedback for correct diagnostic info
    TrajectoryPoint fb;
    bool found_error = false;
    for(int i = 0; i < 20; ++i) { // Try for 2 seconds
        if (mm->dequeueFeedback(fb)) {
            if (fb.diagnostics.safety == SafetyStatus::Error_JointLimit) {
                found_error = true;
                break;
            }
        }
        std::this_thread::sleep_for(100ms);
    }
    EXPECT_TRUE(found_error);
    EXPECT_EQ(fb.diagnostics.safety, SafetyStatus::Error_JointLimit);
    EXPECT_EQ(fb.diagnostics.failing_axis_id, 0); // Axis A1

    // Queues should be cleared
    EXPECT_EQ(mm->getCommandQueueSize(), 0);
}

// Test Reset functionality after an error
TEST_F(MotionManagerTest, ResetsFromErrorState) {
    mm = std::make_unique<MotionManager>(hw_manager, 100, limits, 5.0_deg);
    ASSERT_TRUE(mm->start());
    
    // Force an error state
    mm->emergencyStop();
    ASSERT_EQ(mm->getCurrentState(), RTState::Error);

    // Reset
    mm->reset();
    ASSERT_EQ(mm->getCurrentState(), RTState::Idle);

    // Check if it can execute a new command
    TrajectoryPoint valid_target;
    valid_target.command.joint_target.SetFromPositionArray({5.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    ASSERT_TRUE(mm->enqueueCommand(valid_target));
    
    // Wait for motion to start and complete (at least one tick of 100ms)
    std::this_thread::sleep_for(150ms); 
    EXPECT_TRUE(waitForState(mm, RTState::Idle, 1000ms));
    
    auto fb_res = hw_manager->read();
    ASSERT_TRUE(fb_res.isSuccess());
    EXPECT_NEAR(fb_res.value().joints.GetAt(0).value().get().position.value(), 5.0, 0.01);
}

// Test Following Error detection
TEST_F(MotionManagerTest, DetectsFollowingError) {
    // We trigger a following error by setting very low velocity limits in HAL.
    // When MM sends a large jump, the HAL Governor will clamp it, creating a lag.
    
    auto slow_limits = limits;
    slow_limits.joint_velocity_limits_deg_s.fill(1.0_deg_s); // 1 deg/sec is very slow
    
    // Re-create hw_manager with slow limits
    hw_manager = std::make_shared<HardwareManager>(config, slow_limits);
    ASSERT_TRUE(hw_manager->init().isSuccess());

    mm = std::make_unique<MotionManager>(hw_manager, 100, slow_limits, 2.0_deg); // 2 degree error threshold
    ASSERT_TRUE(mm->start());
    
    TrajectoryPoint target;
    // 1. Send an initial point at 0,0 to initialize the Governor in HAL
    target.command.joint_target.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    ASSERT_TRUE(mm->enqueueCommand(target));
    std::this_thread::sleep_for(150ms); // Wait for at least one tick

    // 2. Now send a large jump (10 degrees). 
    // Since is_first_cmd is now false, the Governor will clamp this jump to 1 deg/sec.
    // In 100ms cycle, it will only move 0.1 deg. 
    // Command is 10.0, Feedback is 0.1. Error 9.9 > Threshold 2.0.
    target.command.joint_target.SetFromPositionArray({10.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    ASSERT_TRUE(mm->enqueueCommand(target));

    // MM should detect the lag and enter error state
    EXPECT_TRUE(waitForState(mm, RTState::Error, 1000ms));

    // Check the feedback for the correct diagnostic
    TrajectoryPoint fb;
    bool error_found = false;
    while(mm->dequeueFeedback(fb)) {
        if (fb.diagnostics.safety == SafetyStatus::Error_FollowingError) {
            EXPECT_EQ(fb.diagnostics.failing_axis_id, 0);
            error_found = true;
            break;
        }
    }
    EXPECT_TRUE(error_found);
}

int main(int argc, char **argv) {
    RDT::Logger::Init({std::make_shared<RDT::ConsoleSink>()}, RDT::LogLevel::Info);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    RDT::Logger::Shutdown();
    return result;
}
