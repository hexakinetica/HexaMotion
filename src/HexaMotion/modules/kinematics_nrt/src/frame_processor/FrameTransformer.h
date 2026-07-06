// FrameTransformer.h
#ifndef FRAME_TRANSFORMER_H
#define FRAME_TRANSFORMER_H

#pragma once

#include "DataTypes.h"
#include "Units.h"

namespace RDT {

/**
 * @class FrameTransformer
 * @brief Named facade for coordinate frame transformations.
 * Provides static methods for transforming poses between different coordinate frames,
 * such as calculating TCP pose from flange pose. The math is delegated to the shared single source
 * of truth RDT::pose_math (shared/data_types/PoseMath.h); KDL is no longer used here (it remains
 * only in the FK/IK solver).
 * @version 3.0 (Delegates to shared PoseMath)
 */
class FrameTransformer {
public:
    FrameTransformer() = delete; // Static class

    /**
     * @brief Calculates TCP (Tool Center Point) pose in world/base frame.
     * Transformation: TCP_in_World = Flange_in_World * Tool_Transform
     * @param flange_pose The flange pose in the world/base frame.
     * @param tool_transform The tool frame's transformation (offset from flange to TCP).
     * @return The calculated TCP pose in the world/base frame.
     */
    [[nodiscard]] static CartPose calculateTcpInWorld(const CartPose& flange_pose, const CartPose& tool_transform);

    /**
     * @brief Calculates flange pose in world/base frame from TCP pose (inverse operation).
     * Transformation: Flange_in_World = TCP_in_World * (Tool_Transform)^-1
     * @param tcp_pose The TCP pose in the world/base frame.
     * @param tool_transform The tool frame's transformation (offset from flange to TCP).
     * @return The calculated flange pose in the world/base frame.
     */
    [[nodiscard]] static CartPose calculateFlangeInWorld(const CartPose& tcp_pose, const CartPose& tool_transform);

    /**
     * @brief Applies a base transformation to a given pose.
     * Transformation: Result_in_World = Base_Transform * Pose_in_Base
     * @param base_transform The transformation from the world frame to the base frame.
     * @param pose The pose defined within the base frame.
     * @return The resulting pose expressed in the world frame.
     */
    [[nodiscard]] static CartPose applyBaseTransform(const CartPose& base_transform, const CartPose& pose);

    /**
     * @brief Transforms a world pose into the base frame.
     * Transformation: Pose_in_Base = (Base_Transform)^-1 * Pose_in_World
     * @param base_transform The base transform (base frame in world coordinates).
     * @param pose_world The pose expressed in world coordinates.
     * @return The pose expressed in the base frame.
     */
    [[nodiscard]] static CartPose applyBaseInverseTransform(const CartPose& base_transform, const CartPose& pose_world);

    /**
     * @brief Composes two poses (left-multiply).
     * Transformation: Result = A * B
     * @param lhs The left-hand pose.
     * @param rhs The right-hand pose.
     * @return The composed pose.
     */
    [[nodiscard]] static CartPose composePoses(const CartPose& lhs, const CartPose& rhs);

    /**
     * @brief Rotates a pose by `delta` about a fixed frame axis (world/base), keeping position.
     *
     * The rotation is applied by PRE-multiplying a single-axis rotation onto the pose's current
     * orientation (extrinsic): R' = R_axis(delta) * R_pose. This is the correct operation for
     * world/base orientation jog: it rotates about the fixed frame axis regardless of the current
     * wrist orientation. It deliberately replaces the previous approach of adding `delta` onto an
     * Euler (RPY) component, which is only equivalent to an axis rotation when the other two Euler
     * angles are zero and otherwise rotates about the wrong axis.
     *
     * Delegates to RDT::pose_math::rotateAboutFrameAxis, which composes the rotation in the shared
     * frame algebra and serializes back to RPY in the single project convention (Rz*Ry*Rx), matching
     * the rest of the pipeline (solveFK).
     *
     * @param pose The pose to rotate (orientation in RPY degrees; position in millimeters).
     * @param axis_index The fixed frame axis to rotate about: 0 = X, 1 = Y, 2 = Z.
     * @param delta The rotation magnitude in degrees (signed).
     * @return The rotated pose. Position is unchanged. An invalid axis_index returns `pose` as-is.
     */
    [[nodiscard]] static CartPose rotateAboutWorldAxis(const CartPose& pose, int axis_index, Degrees delta);
};

} // namespace RDT

#endif // FRAME_TRANSFORMER_H