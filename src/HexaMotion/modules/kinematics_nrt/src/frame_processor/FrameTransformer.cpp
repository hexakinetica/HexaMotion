// FrameTransformer.cpp
#include "FrameTransformer.h"
#include "PoseMath.h"

namespace RDT {

// FrameTransformer is a thin, named facade over the shared pose_math frame algebra
// (shared/data_types/src/PoseMath.h). The math lives in exactly one place and is used by both
// HexaMotion (here) and HexaStudio (RobotService monitor/jog displays), so the two cannot diverge.
// KDL is intentionally NOT used here anymore; it remains only in the FK/IK solver where it is
// required. The orientation convention (RPY = Rz*Ry*Rx) is defined once in PoseMath.

CartPose FrameTransformer::calculateTcpInWorld(const CartPose& flange_pose, const CartPose& tool_transform) {
    // TCP_in_World = Flange_in_World * Tool_Transform
    return pose_math::compose(flange_pose, tool_transform);
}

CartPose FrameTransformer::calculateFlangeInWorld(const CartPose& tcp_pose, const CartPose& tool_transform) {
    // Flange_in_World = TCP_in_World * (Tool_Transform)^-1
    return pose_math::compose(tcp_pose, pose_math::inverse(tool_transform));
}

CartPose FrameTransformer::applyBaseTransform(const CartPose& base_transform, const CartPose& pose) {
    // Result_in_World = Base_Transform * Pose_in_Base
    return pose_math::compose(base_transform, pose);
}

CartPose FrameTransformer::applyBaseInverseTransform(const CartPose& base_transform, const CartPose& pose_world) {
    // Pose_in_Base = (Base_Transform)^-1 * Pose_in_World
    return pose_math::compose(pose_math::inverse(base_transform), pose_world);
}

CartPose FrameTransformer::composePoses(const CartPose& lhs, const CartPose& rhs) {
    // Result = A * B
    return pose_math::compose(lhs, rhs);
}

CartPose FrameTransformer::rotateAboutWorldAxis(const CartPose& pose, int axis_index, Degrees delta) {
    // Extrinsic rotation about the fixed frame axis (pre-multiply), position unchanged.
    return pose_math::rotateAboutFrameAxis(pose, axis_index, delta);
}

} // namespace RDT
