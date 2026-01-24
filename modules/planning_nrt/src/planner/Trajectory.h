// Trajectory.h
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#pragma once

#include "segments/MotionSegment.h"
#include <deque>
#include <memory>
#include <mutex>

namespace RDT {

/**
 * @class Trajectory
 * @brief A thread-safe queue of MotionSegment objects.
 *
 * This class represents the complete planned path for the robot, which may consist
 * of multiple sequential motion segments. It provides thread-safe access for the
 * planner to add new segments and to consume points from the active segment.
 */
class Trajectory {
public:
    Trajectory() = default;

    /** @brief Adds a new segment to the back of the trajectory queue. */
    void addSegment(std::unique_ptr<MotionSegment> segment);

    /**
     * @brief Gets a batch of points from the front of the trajectory.
     * Consumes points from the current active segment. If the active segment is depleted,
     * it automatically moves to the next segment in the queue.
     * @param batch_size The maximum number of points to retrieve.
     * @return A vector containing the next batch of TrajectoryPoints. Can be empty if the trajectory is finished.
     */
    [[nodiscard]] std::vector<TrajectoryPoint> getNextPointBatch(size_t batch_size);

    /** @brief Clears all segments from the trajectory. */
    void clear();

    /** @brief Checks if there are no more points to consume in the entire trajectory. */
    [[nodiscard]] bool isFinished() const;

private:
    mutable std::mutex mutex_;
    std::deque<std::unique_ptr<MotionSegment>> segments_;
    size_t current_point_index_ = 0;
};

} // namespace RDT

#endif // TRAJECTORY_H