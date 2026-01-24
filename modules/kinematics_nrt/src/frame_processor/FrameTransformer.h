// FrameTransformer.h
#ifndef FRAME_TRANSFORMER_H
#define FRAME_TRANSFORMER_H

#pragma once

#include "DataTypes.h"
#include "Units.h"

namespace RDT {

/**
 * @class FrameTransformer
 * @brief Utility class for coordinate frame transformations using KDL.
 * Provides static methods for transforming poses between different coordinate frames,
 * such as calculating TCP pose from flange pose.
 * @version 2.0 (Refactored)
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
};

} // namespace RDT

#endif // FRAME_TRANSFORMER_H