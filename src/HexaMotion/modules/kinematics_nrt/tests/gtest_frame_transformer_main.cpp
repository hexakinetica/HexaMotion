#include "gtest/gtest.h"
#include "frame_processor/FrameTransformer.h"
#include "DataTypes.h"
#include "Units.h"
#include "PoseMath.h"
#include "LoggingMacros.h"

#include <Eigen/Geometry>

using namespace RDT;
using namespace RDT::literals;

// Convert a CartPose orientation to a rotation matrix using the project RPY convention (Rz*Ry*Rx).
// Comparing matrices is representation-independent, so it does not depend on the Euler branch chosen
// when serializing back to RPY.
static Eigen::Matrix3d cartPoseToMatrix(const CartPose& p) {
    return (Eigen::AngleAxisd(p.rz.toRadians(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(p.ry.toRadians(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(p.rx.toRadians(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

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

    // Compare two rotation matrices element-wise.
    void ExpectMatrixNear(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b, double tolerance = 1e-4) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                EXPECT_NEAR(a(i, j), b(i, j), tolerance) << "mismatch at (" << i << "," << j << ")";
            }
        }
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

// --- rotateAboutWorldAxis (orientation jog) -------------------------------------------------

TEST_F(FrameTransformerTest, RotateAboutWorldAxis_FromIdentity_Z90) {
    CartPose start; // identity orientation, non-zero position
    start.x = 10.0_mm; start.y = 20.0_mm; start.z = 30.0_mm;

    CartPose out = FrameTransformer::rotateAboutWorldAxis(start, 2, 90.0_deg);

    // Position must be untouched.
    EXPECT_NEAR(out.x.value(), 10.0, 1e-4);
    EXPECT_NEAR(out.y.value(), 20.0, 1e-4);
    EXPECT_NEAR(out.z.value(), 30.0, 1e-4);
    // From identity, a clean +90 deg about Z is unambiguous in RPY.
    EXPECT_NEAR(out.rz.value(), 90.0, 1e-4);
    EXPECT_NEAR(out.ry.value(), 0.0, 1e-4);
    EXPECT_NEAR(out.rx.value(), 0.0, 1e-4);
}

TEST_F(FrameTransformerTest, RotateAboutWorldAxis_IndependentOfStartOrientation) {
    // A tilted start pose: this is exactly where the old Euler-addition rotated about the wrong axis.
    CartPose start;
    start.rx = 80.0_deg;
    start.ry = -35.0_deg;
    start.rz = 120.0_deg;
    start.x = 5.0_mm; start.y = -7.0_mm; start.z = 11.0_mm;

    const double deltaDeg = 17.0;
    for (int axis = 0; axis < 3; ++axis) {
        CartPose out = FrameTransformer::rotateAboutWorldAxis(start, axis, Degrees(deltaDeg));

        // Expected: pre-multiply (extrinsic) the world-axis rotation onto the start orientation.
        Eigen::Vector3d a = Eigen::Vector3d::Zero();
        a[axis] = 1.0;
        const Eigen::Matrix3d expected =
            Eigen::AngleAxisd(Degrees(deltaDeg).toRadians(), a).toRotationMatrix() * cartPoseToMatrix(start);

        ExpectMatrixNear(cartPoseToMatrix(out), expected);
        // Position is never changed by an orientation jog.
        EXPECT_NEAR(out.x.value(), 5.0, 1e-4);
        EXPECT_NEAR(out.y.value(), -7.0, 1e-4);
        EXPECT_NEAR(out.z.value(), 11.0, 1e-4);
    }
}

TEST_F(FrameTransformerTest, RotateAboutWorldAxis_DiffersFromEulerAddition) {
    // Regression guard: at a tilted pose, naive Euler addition (rx += d) is NOT a world-axis
    // rotation, so the resulting orientations must differ.
    CartPose start;
    start.rx = 80.0_deg; start.ry = -35.0_deg; start.rz = 120.0_deg;

    const CartPose proper = FrameTransformer::rotateAboutWorldAxis(start, 0, 10.0_deg);
    CartPose naive = start;
    naive.rx = Degrees(naive.rx.value() + 10.0);

    EXPECT_FALSE(cartPoseToMatrix(proper).isApprox(cartPoseToMatrix(naive), 1e-6));
}

TEST_F(FrameTransformerTest, RotateAboutWorldAxis_InvalidAxisIsNoOp) {
    CartPose start;
    start.rx = 12.0_deg; start.x = 3.0_mm;

    CartPose out = FrameTransformer::rotateAboutWorldAxis(start, 3, 45.0_deg); // axis 3 is invalid

    ExpectPoseNear(out, start);
}

// --- shared RDT::pose_math (single source of truth) -----------------------------------------

TEST_F(FrameTransformerTest, PoseMath_ComposeInverseIsIdentity) {
    CartPose a;
    a.x = 50.0_mm; a.y = -20.0_mm; a.z = 100.0_mm;
    a.rx = 45.0_deg; a.ry = -33.0_deg; a.rz = 120.0_deg;

    // A * A^-1 == identity
    const CartPose id = pose_math::compose(a, pose_math::inverse(a));
    ExpectPoseNear(id, CartPose{});
}

TEST_F(FrameTransformerTest, PoseMath_TcpInFrameIdentityIsFlange) {
    CartPose flange;
    flange.x = 100.0_mm; flange.z = 200.0_mm;
    flange.rx = 30.0_deg; flange.rz = -75.0_deg;

    // Identity tool and base -> TCP equals the flange.
    const CartPose tcp = pose_math::tcpInFrame(flange, CartPose{}, CartPose{});
    ExpectPoseNear(tcp, flange);
}

TEST_F(FrameTransformerTest, PoseMath_TcpInFrameMatchesFrameTransformerComposition) {
    CartPose flange;
    flange.x = 10.0_mm; flange.y = 5.0_mm; flange.z = 250.0_mm;
    flange.rx = 80.0_deg; flange.ry = -35.0_deg; flange.rz = 120.0_deg;

    CartPose tool;  tool.z = 180.0_mm; tool.ry = 90.0_deg;
    CartPose base;  base.x = 1000.0_mm; base.rz = 45.0_deg;

    // tcpInFrame must equal (base^-1 * flange) * tool built from the named FrameTransformer ops.
    const CartPose viaPoseMath = pose_math::tcpInFrame(flange, tool, base);
    const CartPose viaFrameTf = FrameTransformer::composePoses(
        FrameTransformer::applyBaseInverseTransform(base, flange), tool);
    ExpectPoseNear(viaPoseMath, viaFrameTf);
}
