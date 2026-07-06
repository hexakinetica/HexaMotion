#include "gtest/gtest.h"
#include "kinematic_solver/KdlKinematicSolver.h"
#include "KinematicModel.h"
#include "Units.h"
#include "LoggingMacros.h"
#include <filesystem>
#include <cstdlib>

using namespace RDT;
using namespace RDT::literals;

namespace {
std::string resolveUrdfPathForTest() {
    if (const char* env = std::getenv("HEXAMOTION_TEST_URDF")) {
        std::filesystem::path p(env);
        if (std::filesystem::exists(p)) {
            return p.string();
        }
    }

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
    return {};
}

std::unique_ptr<KdlKinematicSolver> buildSolver(RobotLimits& limits) {
    const std::string urdf_path = resolveUrdfPathForTest();
    EXPECT_FALSE(urdf_path.empty());
    RobotModelConfig config;
    config.urdf_path = urdf_path;
    config.base_link = "base_link";
    config.tip_link = "flange";
    auto model = KinematicModel::createFromURDFFile(config);
    limits.joint_position_limits_deg = {{
        {-170.0_deg, 170.0_deg}, {-120.0_deg, 120.0_deg}, {-170.0_deg, 170.0_deg},
        {-120.0_deg, 120.0_deg}, {-170.0_deg, 170.0_deg}, {-175.0_deg, 175.0_deg}
    }};
    limits.joint_velocity_limits_deg_s.fill(100.0_deg_s);
    return std::make_unique<KdlKinematicSolver>(std::move(model), limits);
}
}

TEST(IkIsolated, RoundTrip) {
    RobotLimits limits;
    auto solver = buildSolver(limits);

    AxisSet seed;
    seed.SetFromPositionArray({0.0_deg, -90.0_deg, 0.0_deg, 0.0_deg, 90.0_deg, 0.0_deg});

    CartPose fk_pose;
    ASSERT_TRUE(solver->solveFK(seed, fk_pose));

    auto ik_res = solver->solveIK(fk_pose, seed);
    ASSERT_TRUE(ik_res.isSuccess()) << "IK failed: " << ToString(ik_res.error());

    CartPose check_pose;
    ASSERT_TRUE(solver->solveFK(ik_res.value(), check_pose));
    EXPECT_NEAR(check_pose.x.value(), fk_pose.x.value(), 1e-2);
    EXPECT_NEAR(check_pose.y.value(), fk_pose.y.value(), 1e-2);
    EXPECT_NEAR(check_pose.z.value(), fk_pose.z.value(), 1e-2);
}

TEST(IkIsolated, ToolOffsetExplodesPose) {
    RobotLimits limits;
    auto solver = buildSolver(limits);

    AxisSet seed;
    seed.SetFromPositionArray({0.0_deg, -90.0_deg, 0.0_deg, 0.0_deg, 90.0_deg, 0.0_deg});

    CartPose base_pose;
    ASSERT_TRUE(solver->solveFK(seed, base_pose));

    // Artificially add a huge tool offset (1m) to show unreachable IK
    CartPose unreachable = base_pose;
    unreachable.z += 1000.0_mm;

    auto ik_res = solver->solveIK(unreachable, seed);
    EXPECT_TRUE(ik_res.isError()) << "IK unexpectedly succeeded for unreachable pose.";
}

TEST(IkIsolated, OrientationSensitivity) {
    RobotLimits limits;
    auto solver = buildSolver(limits);

    AxisSet seed;
    seed.SetFromPositionArray({0.0_deg, -90.0_deg, 0.0_deg, 0.0_deg, 90.0_deg, 0.0_deg});

    CartPose pose;
    ASSERT_TRUE(solver->solveFK(seed, pose));

    CartPose rotated = pose;
    rotated.rx += 180.0_deg; // flip X orientation to stress IK

    auto ik_res = solver->solveIK(rotated, seed);
    if (ik_res.isError()) {
        SUCCEED() << "IK failed as expected for aggressive orientation flip.";
    } else {
        CartPose check;
        ASSERT_TRUE(solver->solveFK(ik_res.value(), check));
        EXPECT_NEAR(check.x.value(), rotated.x.value(), 1e-2);
        EXPECT_NEAR(check.y.value(), rotated.y.value(), 1e-2);
        EXPECT_NEAR(check.z.value(), rotated.z.value(), 1e-2);
    }
}

int main(int argc, char** argv) {
    RDT::Logger::Init({std::make_shared<RDT::ConsoleSink>()}, RDT::LogLevel::Info);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    RDT::Logger::Shutdown();
    return result;
}