// TrajectoryPlanner.h
#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#pragma once

#include "segments/TrajectoryInterpolator.h"
#include "planner/Trajectory.h"
#include "MotionManager.h"

namespace RDT {

/**
 * @class TrajectoryPlanner
 * @brief The high-level manager for trajectory generation and execution.
 *
 * This class orchestrates the entire planning process. It receives high-level
 * motion commands, uses a `TrajectoryInterpolator` to create `MotionSegment`s,
 * manages them in a `Trajectory` queue, and feeds the resulting points to the
 * `MotionManager`'s buffer.
 * @version 2.0 (Refactored)
 */
class TrajectoryPlanner {
public:
    using PlannerError = TrajectoryInterpolator::PlannerError;

    /**
     * @brief Constructs the TrajectoryPlanner.
     * @param interpolator A shared pointer to the segment factory.
     * @param motion_manager A shared pointer to the RT motion executor.
     */
    TrajectoryPlanner(std::shared_ptr<TrajectoryInterpolator> interpolator,
                      std::shared_ptr<MotionManager> motion_manager);
    
    /**
     * @brief Sets the current state of the robot, which serves as the starting point for the next motion.
     * @param current_robot_state The current robot state.
     */
    void setCurrentState(const TrajectoryPoint& current_robot_state);

    /**
     * @brief Creates a motion segment to a new target and adds it to the trajectory queue.
     * This method supports both streaming points and building a program trajectory.
     * @param target_waypoint The next waypoint to move to.
     * @return A Result indicating success or the reason for a planning failure.
     */
    [[nodiscard]] Result<void, PlannerError> addTargetWaypoint(const TrajectoryPoint& target_waypoint);
    
    /**
     * @brief Immediately clears all planned and buffered motion and generates a new trajectory from the current state.
     * @param target_waypoint The new, urgent target.
     * @return A Result indicating success or failure.
     */
    [[nodiscard]] Result<void, PlannerError> overrideTrajectory(const TrajectoryPoint& target_waypoint);

    /**
     * @brief Main update loop for the planner.
     * This non-blocking method should be called periodically from the main application loop.
     * It checks the buffer level of the MotionManager and feeds it new points from the trajectory.
     */
    void update();

    /** @brief Checks if the entire planned trajectory has been sent to the MotionManager. */
    [[nodiscard]] bool isTaskFinished() const;

private:
    std::shared_ptr<TrajectoryInterpolator> interpolator_;
    std::shared_ptr<MotionManager> motion_manager_;
    
    Trajectory trajectory_;
    TrajectoryPoint current_state_;

    Seconds planning_dt_ = 0.004_s; // Default timestep for segment generation (250Hz)
    
    static inline const std::string MODULE_NAME = "TrajPlanner";
};

} // namespace RDT

#endif // TRAJECTORY_PLANNER_H
