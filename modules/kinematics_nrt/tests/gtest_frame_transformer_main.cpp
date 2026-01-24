#include "gtest/gtest.h"
#include "frame_processor/FrameTransformer.h"
#include "DataTypes.h"
#include "Units.h"
#include "LoggingMacros.h"

using namespace RDT;
using namespace RDT::literals;

class FrameTransformerTest : public ::testing::Test {
protected:
    // Helper to compare CartPoses with tolerance
    void ExpectPoseNear(const CartPose& actual, const CartPose& expected, double tolerance = 1e-4) {
        EXPECT_NEAR(actual.x.value(), expected.x.value(), tolerance);
        EXPECT_NEAR(actual.y.value(), expected.y.value(), tolerance);
        EXPECT_NEAR(actual.z.value(), expected.z.value(), tolerance);
        EXPECT_NEAR(actual.rx.value(), expected.rx.value(), tolerance);
        EXPECT_NEAR(actual.ry.value(), expected.ry.value(), tolerance);
        EXPECT_NEAR(actual.rz.value(), expected.rz.value(), tolerance);
    }
};

TEST_F(FrameTransformerTest, IdentityTransform) {
    CartPose flange_pose;
    flange_pose.x = 100.0_mm;
    flange_pose.y = 200.0_mm;
    flange_pose.z = 300.0_mm;
    flange_pose.rx = 10.0_deg;
    
    CartPose tool_transform; // Identity by default (all zeros)

    CartPose tcp_pose = FrameTransformer::calculateTcpInWorld(flange_pose, tool_transform);
    
    ExpectPoseNear(tcp_pose, flange_pose);
}

TEST_F(FrameTransformerTest, PureTranslation) {
    CartPose flange_pose; // Identity
    CartPose tool_transform;
    tool_transform.z = 100.0_mm;

    CartPose tcp_pose = FrameTransformer::calculateTcpInWorld(flange_pose, tool_transform);
    
    EXPECT_NEAR(tcp_pose.z.value(), 100.0, 1e-4);
    EXPECT_NEAR(tcp_pose.x.value(), 0.0, 1e-4);
}

TEST_F(FrameTransformerTest, PureRotation) {
    CartPose flange_pose; // Identity
    
    CartPose tool_transform;
    tool_transform.rz = 90.0_deg; // Rotate 90 around Z

    CartPose tcp_pose = FrameTransformer::calculateTcpInWorld(flange_pose, tool_transform);
    
    // Position should be same (0)
    EXPECT_NEAR(tcp_pose.x.value(), 0.0, 1e-4);
    // Rotation should be 90 deg around Z
    EXPECT_NEAR(tcp_pose.rz.value(), 90.0, 1e-4);
}

TEST_F(FrameTransformerTest, CombinedTransform) {
    // Flange is at 100mm X
    CartPose flange_pose;
    flange_pose.x = 100.0_mm;
    
    // Tool is offset by 50mm Z and rotated 90 deg Y
    CartPose tool_transform;
    tool_transform.z = 50.0_mm;
    tool_transform.ry = 90.0_deg;

    // Result should be 100mm X, 50mm Z, 90 deg Y
    CartPose tcp_pose = FrameTransformer::calculateTcpInWorld(flange_pose, tool_transform);
    
    EXPECT_NEAR(tcp_pose.x.value(), 100.0, 1e-4);
    EXPECT_NEAR(tcp_pose.z.value(), 50.0, 1e-4);
    EXPECT_NEAR(tcp_pose.ry.value(), 90.0, 1e-4);
}

TEST_F(FrameTransformerTest, InverseRoundTrip) {
    CartPose flange_pose;
    flange_pose.x = 50.0_mm;
    flange_pose.y = -20.0_mm;
    flange_pose.z = 100.0_mm;
    flange_pose.rx = 45.0_deg;

    CartPose tool_transform;
    tool_transform.z = 200.0_mm;
    tool_transform.ry = 10.0_deg;

    // Forward
    CartPose tcp_pose = FrameTransformer::calculateTcpInWorld(flange_pose, tool_transform);
    
    // Inverse
    CartPose recovered_flange = FrameTransformer::calculateFlangeInWorld(tcp_pose, tool_transform);

    ExpectPoseNear(recovered_flange, flange_pose);
}

TEST_F(FrameTransformerTest, ApplyBaseTransform) {
    CartPose base_transform;
    base_transform.x = 1000.0_mm; // Base is shifted 1m in X
    
    CartPose pose_in_base;
    pose_in_base.x = 100.0_mm; // Point is 100mm in X relative to base

    CartPose pose_in_world = FrameTransformer::applyBaseTransform(base_transform, pose_in_base);
    
    EXPECT_NEAR(pose_in_world.x.value(), 1100.0, 1e-4);
}
