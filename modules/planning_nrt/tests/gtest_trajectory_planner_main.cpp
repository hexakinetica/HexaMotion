#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "planner/TrajectoryPlanner.h"
#include "MotionManager.h" 
#include "HardwareManager.h"
#include <thread>
#include <chrono>

using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;

namespace {
class LocalMockKinematicSolver : public KinematicSolver {
public:
    MOCK_METHOD(bool, solveFK, (const AxisSet& joints, CartPose& result), (const, override));
    MOCK_METHOD((Result<AxisSet, IKError>), solveIK, (const CartPose& pose, const AxisSet& seed_joints), (const, override));
    MOCK_METHOD(void, setHomePosition, (const AxisSet& home_joints), (override));
    MOCK_METHOD(AxisSet, getHomePosition, (), (const, override));
};
}

// A test fixture to set up the full stack: Planner -> MotionManager -> HardwareManager (sim)
class TrajectoryPlannerIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Kinematics
        auto solver = std::make_shared<LocalMockKinematicSolver>();
        
        // Setup default mock behaviors
        ON_CALL(*solver, solveIK)
            .WillByDefault([](const CartPose& pose, const AxisSet& seed) {
                // Simple mock IK: just return the seed
                (void)pose;
                return Result<AxisSet, IKError>::Success(seed);
            });
        ON_CALL(*solver, solveFK)
            .WillByDefault([](const AxisSet& joints, CartPose& result) {
                // Simple mock FK
                (void)joints;
                result = CartPose{};
                return true;
            });
            
        // HAL
        limits.joint_position_limits_deg.fill({-180.0_deg, 180.0_deg});
        limits.joint_velocity_limits_deg_s.fill(100.0_deg_s);
        auto hw_manager = std::make_shared<HardwareManager>(config, limits);
        ASSERT_TRUE(hw_manager->init().isSuccess());

        // Motion Manager
        // Increase cycle time to 40ms for debugging visibility
        motion_manager = std::make_shared<MotionManager>(hw_manager, 40, limits, 20.0_deg);
        ASSERT_TRUE(motion_manager->start());

        // Planner
        auto interpolator = std::make_shared<TrajectoryInterpolator>(solver);
        planner = std::make_unique<TrajectoryPlanner>(interpolator, motion_manager);

        // Set initial state
        TrajectoryPoint initial_state;
        planner->setCurrentState(initial_state);
    }

    void TearDown() override {
        motion_manager->stop();
    }

    InterfaceConfig config;
    RobotLimits limits;
    std::shared_ptr<MotionManager> motion_manager;
    std::unique_ptr<TrajectoryPlanner> planner;
};

TEST_F(TrajectoryPlannerIntegrationTest, AddSingleTargetAndExecute) {
    TrajectoryPoint target;
    target.header.motion_type = MotionType::JOINT;
    target.command.joint_target.SetFromPositionArray({10.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    target.command.speed_ratio = 0.1;

    // Plan the motion
    auto plan_res = planner->addTargetWaypoint(target);
    ASSERT_TRUE(plan_res.isSuccess());
    RDT_LOG_INFO("Test", "Waypoint added. Starting update loop.");

    // Run the planner's update loop until the task is finished
    int loop_count = 0;
    while (!planner->isTaskFinished() && loop_count < 500) {
        planner->update();
        
        // Always dequeue feedback to clear the queue, but only log periodically
        TrajectoryPoint fb;
        bool has_fb = false;
        while(motion_manager->dequeueFeedback(fb)) {
            has_fb = true;
        }

        if (loop_count % 10 == 0) {
            if (has_fb) { 
               RDT_LOG_INFO("Test", "Loop {}: RT State: {}, Target Pos: {}, Actual Pos: {}", 
                    loop_count, 
                    (int)motion_manager->getCurrentState(),
                    fb.command.joint_target.GetAt(0).value().get().position.value(),
                    fb.feedback.joint_actual.GetAt(0).value().get().position.value());
            } else {
                RDT_LOG_INFO("Test", "Loop {}: RT State: {}. No feedback.", 
                    loop_count, (int)motion_manager->getCurrentState());
            }
        }
        std::this_thread::sleep_for(100ms); // Increased delay for debugging
        loop_count++;
    }
    
    RDT_LOG_INFO("Test", "Update loop finished at loop {}. RT State: {}", loop_count, (int)motion_manager->getCurrentState());
    ASSERT_TRUE(planner->isTaskFinished());
    EXPECT_GT(motion_manager->getFeedbackQueueSize(), 0);

    // Check final state
    TrajectoryPoint last_fb;
    int fb_count = 0;
    while(motion_manager->dequeueFeedback(last_fb)) { fb_count++; } // Drain queue to get the last point
    RDT_LOG_INFO("Test", "Drained {} feedback points. Final pos: {}", fb_count, last_fb.feedback.joint_actual.GetAt(0).value().get().position.value());
    
    EXPECT_EQ(last_fb.feedback.rt_state, RTState::Idle);
    EXPECT_NEAR(last_fb.feedback.joint_actual.GetAt(0).value().get().position.value(), 10.0, 0.05);
}

TEST_F(TrajectoryPlannerIntegrationTest, StreamingTargetsAreQueued) {
    // Target 1
    TrajectoryPoint target1;
    target1.header.motion_type = MotionType::JOINT;
    target1.command.joint_target.SetFromPositionArray({10.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    target1.command.speed_ratio = 0.1;

    // Target 2
    TrajectoryPoint target2;
    target2.header.motion_type = MotionType::JOINT;
    target2.command.joint_target.SetFromPositionArray({20.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    target2.command.speed_ratio = 0.1;

    // Plan both motions back-to-back
    ASSERT_TRUE(planner->addTargetWaypoint(target1).isSuccess());
    ASSERT_TRUE(planner->addTargetWaypoint(target2).isSuccess());

    // Execute
    int loop_count = 0;
    while (!planner->isTaskFinished() && loop_count < 1000) {
        planner->update();
        std::this_thread::sleep_for(100ms); // Increased delay for debugging
        loop_count++;
    }

    ASSERT_TRUE(planner->isTaskFinished());

    TrajectoryPoint last_fb;
    while(motion_manager->dequeueFeedback(last_fb)) {}
    
    EXPECT_EQ(last_fb.feedback.rt_state, RTState::Idle);
    EXPECT_NEAR(last_fb.feedback.joint_actual.GetAt(0).value().get().position.value(), 20.0, 0.01);
}

int main(int argc, char **argv) {
    RDT::Logger::Init({std::make_shared<RDT::ConsoleSink>()}, RDT::LogLevel::Info);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    RDT::Logger::Shutdown();
    return result;
}
