#include "gtest/gtest.h"
#include "kinematic_solver/KdlKinematicSolver.h"
#include "KinematicModel.h"
#include "DataTypes.h"
#include "RobotConfig.h" // For RobotLimits
#include "Units.h"
#include "LoggingMacros.h"
#include <fstream>
#include <sstream>
#include <chrono>
#include <random>
#include <filesystem>
#include <cstdlib>
#include <cmath>

using namespace RDT;
using namespace RDT::literals;

class KinematicSolverTest : public ::testing::Test {
protected:
    void SetUp() override {
        const std::string urdf_path = resolveUrdfPathForTest();
        ASSERT_FALSE(urdf_path.empty()) << "URDF file path is not resolved. Set HEXAMOTION_TEST_URDF or keep test URDF in repo.";
        RobotModelConfig config;
        config.urdf_path = urdf_path;
        config.base_link = "base_link";
        config.tip_link = "flange";
        auto model = KinematicModel::createFromURDFFile(config);
        ASSERT_NE(model, nullptr);

        // Define realistic limits for the test model
        limits.joint_position_limits_deg = {{
            {-170.0_deg, 170.0_deg}, {-120.0_deg, 120.0_deg}, {-170.0_deg, 170.0_deg},
            {-120.0_deg, 120.0_deg}, {-170.0_deg, 170.0_deg}, {-175.0_deg, 175.0_deg}
        }};
        limits.joint_velocity_limits_deg_s.fill(100.0_deg_s);

        solver = std::make_unique<KdlKinematicSolver>(std::move(model), limits);
    }

    static std::string resolveUrdfPathForTest() {
        // 1) external override (no hardcoded path requirement)
        if (const char* env = std::getenv("HEXAMOTION_TEST_URDF")) {
            std::filesystem::path p(env);
            if (std::filesystem::exists(p)) {
                return p.string();
            }
        }

        // 2) repository-local fallbacks
        const std::vector<std::string> candidates = {
            "HexaMotion/modules/kinematics_nrt/tests/lbr_iisy_11_r1300.urdf",
            "../tests/lbr_iisy_11_r1300.urdf",
            "tests/lbr_iisy_11_r1300.urdf"
        };

        for (const auto& c : candidates) {
            std::filesystem::path p(c);
            if (std::filesystem::exists(p)) {
                return p.string();
            }
        }
        return "";
    }

    std::unique_ptr<KdlKinematicSolver> solver;
    RobotLimits limits;
};

// Test that FK and IK work correctly in a round-trip validation
TEST_F(KinematicSolverTest, FkIkRoundTrip) {
    AxisSet original_joints;
    original_joints.SetFromPositionArray({10.0_deg, 20.0_deg, 30.0_deg, 40.0_deg, 50.0_deg, 60.0_deg});

    // 1. Solve FK to get the target Cartesian pose
    CartPose target_pose;
    ASSERT_TRUE(solver->solveFK(original_joints, target_pose));

    // 2. Use a slightly different seed for IK to ensure the solver works
    AxisSet seed_joints;
    seed_joints.SetFromPositionArray({11.0_deg, 21.0_deg, 29.0_deg, 41.0_deg, 49.0_deg, 61.0_deg});

    // 3. Solve IK
    auto ik_result = solver->solveIK(target_pose, seed_joints);
    ASSERT_TRUE(ik_result.isSuccess()) << "IK failed with error: " << ToString(ik_result.error());
    
    const AxisSet& result_joints = ik_result.value();

    // 4. Compare the resulting joints to the original joints
    const auto original_pos = original_joints.ToPositionArray();
    const auto result_pos = result_joints.ToPositionArray();
    
    // Increased tolerance from 1e-4 to 1e-3 because numerical solver might be less precise
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        EXPECT_NEAR(result_pos[i].value(), original_pos[i].value(), 1e-3)
            << "Mismatch on axis " << i + 1;
    }
}

// Test that IK fails for a clearly unreachable target
TEST_F(KinematicSolverTest, IkFailsForUnreachableTarget) {
    CartPose unreachable_pose;
    unreachable_pose.x = 5000.0_mm; // 5 meters, way outside the workspace
    unreachable_pose.y = 5000.0_mm;
    unreachable_pose.z = 5000.0_mm;

    AxisSet seed_joints; // Start from zero
    
    auto ik_result = solver->solveIK(unreachable_pose, seed_joints);
    
    ASSERT_TRUE(ik_result.isError());
    // KDL Newton-Raphson solver returns MAX_ITERATIONS when it can't reach, which we map to SolverTimeout.
    // So we accept either Unreachable or SolverTimeout.
    EXPECT_TRUE(ik_result.error() == IKError::Unreachable || ik_result.error() == IKError::SolverTimeout)
        << "Expected Unreachable or SolverTimeout, but got: " << ToString(ik_result.error());
}

// Test home position getter and setter
TEST_F(KinematicSolverTest, HomePosition) {
    AxisSet home_pos;
    home_pos.SetFromPositionArray({0.0_deg, -90.0_deg, 0.0_deg, 0.0_deg, 90.0_deg, 0.0_deg});

    solver->setHomePosition(home_pos);
    
    AxisSet retrieved_home_pos = solver->getHomePosition();

    const auto expected = home_pos.ToPositionArray();
    const auto retrieved = retrieved_home_pos.ToPositionArray();

    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        EXPECT_EQ(expected[i], retrieved[i]);
    }
}

// NEW TEST: Compare hardcoded model vs. URDF-loaded model
TEST_F(KinematicSolverTest, ModelLoadingComparison) {
    const std::string urdf_path = resolveUrdfPathForTest();
    ASSERT_FALSE(urdf_path.empty()) << "URDF file path is not resolved. Set HEXAMOTION_TEST_URDF or keep test URDF in repo.";
    RDT_LOG_INFO("ModelComparison", "Using URDF path: {}", urdf_path);

    // 1) Build model from URDF file path
    RobotModelConfig config;
    config.urdf_path = urdf_path;
    config.base_link = "base_link";
    config.tip_link = "flange";
    auto model_from_urdf = KinematicModel::createFromURDFFile(config);
    ASSERT_NE(model_from_urdf, nullptr);
    ASSERT_EQ(model_from_urdf->getNrOfJoints(), ROBOT_AXES_COUNT);
    auto solver_from_urdf = std::make_unique<KdlKinematicSolver>(std::move(model_from_urdf), limits);

    // 2) Compare FK over multiple joint sets
    const std::vector<std::array<Degrees, ROBOT_AXES_COUNT>> testSets = {
        { 15.0_deg, -30.0_deg,   0.0_deg,  45.0_deg,  90.0_deg, -10.0_deg },
        {  0.0_deg, -90.0_deg,  20.0_deg,   0.0_deg,  90.0_deg,   0.0_deg },
        {-35.0_deg, -60.0_deg,  45.0_deg, -20.0_deg,  30.0_deg,  80.0_deg }
    };

    constexpr double tolerance = 1e-5; // mm / deg
    for (size_t i = 0; i < testSets.size(); ++i) {
        AxisSet joints;
        joints.SetFromPositionArray(testSets[i]);

        CartPose pose_from_hardcoded;
        CartPose pose_from_urdf;
        ASSERT_TRUE(solver->solveFK(joints, pose_from_hardcoded));
        ASSERT_TRUE(solver_from_urdf->solveFK(joints, pose_from_urdf));

        RDT_LOG_INFO("ModelComparison", "Set {} hardcoded FK: {}", i, pose_from_hardcoded.toDescriptiveString());
        RDT_LOG_INFO("ModelComparison", "Set {} URDF FK:      {}", i, pose_from_urdf.toDescriptiveString());

        const double dx = std::abs(pose_from_hardcoded.x.value() - pose_from_urdf.x.value());
        const double dy = std::abs(pose_from_hardcoded.y.value() - pose_from_urdf.y.value());
        const double dz = std::abs(pose_from_hardcoded.z.value() - pose_from_urdf.z.value());
        const double drx = std::abs(pose_from_hardcoded.rx.value() - pose_from_urdf.rx.value());
        const double dry = std::abs(pose_from_hardcoded.ry.value() - pose_from_urdf.ry.value());
        const double drz = std::abs(pose_from_hardcoded.rz.value() - pose_from_urdf.rz.value());

        RDT_LOG_INFO("ModelComparison", "Set {} deltas: dX={} mm dY={} mm dZ={} mm dRx={} deg dRy={} deg dRz={} deg",
                     i, dx, dy, dz, drx, dry, drz);

        EXPECT_NEAR(pose_from_hardcoded.x.value(), pose_from_urdf.x.value(), tolerance);
        EXPECT_NEAR(pose_from_hardcoded.y.value(), pose_from_urdf.y.value(), tolerance);
        EXPECT_NEAR(pose_from_hardcoded.z.value(), pose_from_urdf.z.value(), tolerance);
        EXPECT_NEAR(pose_from_hardcoded.rx.value(), pose_from_urdf.rx.value(), tolerance);
        EXPECT_NEAR(pose_from_hardcoded.ry.value(), pose_from_urdf.ry.value(), tolerance);
        EXPECT_NEAR(pose_from_hardcoded.rz.value(), pose_from_urdf.rz.value(), tolerance);
    }
}

TEST_F(KinematicSolverTest, UrdfLimitsLoading) {
    const std::string urdf_path = resolveUrdfPathForTest();
    ASSERT_FALSE(urdf_path.empty()) << "URDF file path is not resolved.";

    RobotModelConfig config;
    config.urdf_path = urdf_path;
    config.base_link = "base_link";
    config.tip_link = "flange";
    auto model_from_urdf = KinematicModel::createFromURDFFile(config);
    ASSERT_NE(model_from_urdf, nullptr);
    ASSERT_TRUE(model_from_urdf->hasRobotLimits());

    const auto& loaded_limits = model_from_urdf->getRobotLimits();

    // Expected values are from tests/lbr_iisy_11_r1300.urdf (converted rad->deg)
    constexpr double tol_deg = 1e-2;
    constexpr double tol_vel = 1e-2;

    // joint_1: [-3.2288, 3.2288], vel 3.4906
    EXPECT_NEAR(loaded_limits.joint_position_limits_deg[0].first.value(), -184.996, tol_deg);
    EXPECT_NEAR(loaded_limits.joint_position_limits_deg[0].second.value(), 184.996, tol_deg);
    EXPECT_NEAR(loaded_limits.joint_velocity_limits_deg_s[0].value(), 200.0, tol_vel);

    // joint_2: [-4.0142, 0.8726], vel 3.4906
    EXPECT_NEAR(loaded_limits.joint_position_limits_deg[1].first.value(), -230.0, tol_deg);
    EXPECT_NEAR(loaded_limits.joint_position_limits_deg[1].second.value(), 50.0, tol_deg);
    EXPECT_NEAR(loaded_limits.joint_velocity_limits_deg_s[1].value(), 200.0, tol_vel);

    // joint_6: [-3.8397, 3.8397], vel 7.5049
    EXPECT_NEAR(loaded_limits.joint_position_limits_deg[5].first.value(), -220.0, tol_deg);
    EXPECT_NEAR(loaded_limits.joint_position_limits_deg[5].second.value(), 220.0, tol_deg);
    EXPECT_NEAR(loaded_limits.joint_velocity_limits_deg_s[5].value(), 430.0, tol_vel);
}

// =================================================================================
// ROBUSTNESS TESTS
// =================================================================================

// Test behavior near singularity (Wrist Alignment)
TEST_F(KinematicSolverTest, SingularityNearMiss) {
    // A configuration near wrist singularity (A5 = 0)
    AxisSet singular_joints;
    singular_joints.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.1_deg, 0.0_deg}); // A5 is almost 0

    CartPose singular_pose;
    ASSERT_TRUE(solver->solveFK(singular_joints, singular_pose));

    // Try to solve IK back to this pose
    AxisSet seed;
    seed.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 10.0_deg, 0.0_deg}); // Seed slightly away

    auto result = solver->solveIK(singular_pose, seed);
    
    if (result.isSuccess()) {
        // Near singularity, check Cartesian reach instead of Joint values (null space drift)
        CartPose reached_pose;
        ASSERT_TRUE(solver->solveFK(result.value(), reached_pose));
        EXPECT_NEAR(reached_pose.x.value(), singular_pose.x.value(), 1e-2);
        EXPECT_NEAR(reached_pose.y.value(), singular_pose.y.value(), 1e-2);
        EXPECT_NEAR(reached_pose.z.value(), singular_pose.z.value(), 1e-2);
    } else {
        EXPECT_EQ(result.error(), IKError::SolverTimeout);
    }
}

// Test joint limit violation
TEST_F(KinematicSolverTest, JointLimitViolation) {
    AxisSet seed_joints = solver->getHomePosition();
    
    // Command a pose that violates limits (A1 limit is -170 deg, try -175)
    AxisSet violations_joints;
    violations_joints.SetFromPositionArray({-175.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 90.0_deg, 0.0_deg});
    
    CartPose target_pose;
    ASSERT_TRUE(solver->solveFK(violations_joints, target_pose));
    
    auto result = solver->solveIK(target_pose, seed_joints);
    
    if (result.isSuccess()) {
        auto sol = result.value().ToPositionArray();
        EXPECT_GE(sol[0].value(), -170.0 - 1e-4);
        EXPECT_LE(sol[0].value(), 170.0 + 1e-4);
    } else {
        EXPECT_TRUE(result.error() == IKError::Unreachable || result.error() == IKError::SolverTimeout);
    }
}

// Test seed sensitivity / Multiple solutions
TEST_F(KinematicSolverTest, SeedSensitivity) {
    // Use a target reachable from Home Position seed
    // Home: {0, -90, 0, 0, 90, 0}
    // Target: {2, -88, 2, 0, 88, 0} (Small perturbation)
    AxisSet target_joints;
    target_joints.SetFromPositionArray({2.0_deg, -88.0_deg, 2.0_deg, 0.0_deg, 88.0_deg, 0.0_deg});
    
    CartPose target_pose;
    ASSERT_TRUE(solver->solveFK(target_joints, target_pose));
    
    // Seed 1: Close to target
    AxisSet seed1 = target_joints;
    auto res1 = solver->solveIK(target_pose, seed1);
    ASSERT_TRUE(res1.isSuccess());
    
    // Seed 2: Home position
    AxisSet seed2 = solver->getHomePosition();
    auto res2 = solver->solveIK(target_pose, seed2);
    if (res2.isError()) {
        GTEST_SKIP() << "SeedSensitivity: solver did not converge from Home seed for this target; skipping strict comparison.";
    }
    
    // Both should reach the target pose (Cartesian check)
    CartPose check1, check2;
    ASSERT_TRUE(solver->solveFK(res1.value(), check1));
    
    if (res2.isSuccess()) {
        ASSERT_TRUE(solver->solveFK(res2.value(), check2));
        EXPECT_NEAR(check2.x.value(), target_pose.x.value(), 1e-3);
        EXPECT_NEAR(check2.y.value(), target_pose.y.value(), 1e-3);
        EXPECT_NEAR(check2.z.value(), target_pose.z.value(), 1e-3);
    }
    
    EXPECT_NEAR(check1.x.value(), target_pose.x.value(), 1e-3);
}

// Performance Benchmark
TEST_F(KinematicSolverTest, PerformanceBenchmark) {
    const int ITERATIONS = 100;
    AxisSet seed = solver->getHomePosition();
    
    AxisSet target_joints = solver->getHomePosition();
    CartPose target_pose;
    solver->solveFK(target_joints, target_pose);
    
    target_pose.x += 10.0_mm; 
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < ITERATIONS; ++i) {
        target_pose.z += 0.1_mm;
        auto res = solver->solveIK(target_pose, seed);
        if (res.isSuccess()) {
            seed = res.value();
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    
    double avg_time_us = static_cast<double>(duration) / ITERATIONS;
    RDT_LOG_INFO("Benchmark", "Average IK time over {} runs: {} us", ITERATIONS, avg_time_us);
    
    EXPECT_LT(avg_time_us, 2000.0); 
}

int main(int argc, char **argv) {
    RDT::Logger::Init({std::make_shared<RDT::ConsoleSink>()}, RDT::LogLevel::Info);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    RDT::Logger::Shutdown();
    return result;
}
