// MotionSegment.h
#ifndef MOTION_SEGMENT_H
#define MOTION_SEGMENT_H

#pragma once

#include "DataTypes.h"
#include "profiles/MotionProfile.h"
#include <vector>
#include <memory>

namespace RDT {

/**
 * @class MotionSegment
 * @brief An immutable, pre-calculated segment of a robot's trajectory.
 *
 * This class represents a single, continuous motion (like one LIN or PTP move).
 * Its key feature is that it contains a cache of all intermediate TrajectoryPoints,
 * fully calculated at the time of its creation. This makes the process of feeding
 * points to the MotionManager a fast and lightweight memory copy operation.
 */
class MotionSegment {
public:
    /**
     * @brief Constructs a MotionSegment.
     * @param points A vector of pre-calculated TrajectoryPoints that form the segment.
     * @param profile A shared pointer to the motion profile used to generate the points.
     */
    MotionSegment(std::vector<TrajectoryPoint>&& points, std::shared_ptr<MotionProfile> profile);

    /** @brief Gets the cached trajectory points for this segment. */
    [[nodiscard]] const std::vector<TrajectoryPoint>& getPoints() const;

    /** @brief Gets the total duration of this segment. */
    [[nodiscard]] Seconds getDuration() const;

    /** @brief Gets the motion type of this segment. */
    [[nodiscard]] MotionType getMotionType() const;

    /** @brief Checks if the segment contains any points. */
    [[nodiscard]] bool isEmpty() const;

private:
    std::vector<TrajectoryPoint> points_;
    std::shared_ptr<MotionProfile> profile_;
};

} // namespace RDT

#endif // MOTION_SEGMENT_H