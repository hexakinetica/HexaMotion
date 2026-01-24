// TrajectoryPlanner.cpp
#include "TrajectoryPlanner.h"
#include "LoggingMacros.h"

namespace RDT {

TrajectoryPlanner::TrajectoryPlanner(std::shared_ptr<TrajectoryInterpolator> interpolator,
                                     std::shared_ptr<MotionManager> motion_manager)
    : interpolator_(interpolator), motion_manager_(motion_manager)
{
    if (!interpolator_ || !motion_manager_) {
        RDT_LOG_CRITICAL(MODULE_NAME, "Interpolator and MotionManager must not be null.");
        throw std::invalid_argument("Interpolator and MotionManager must not be null.");
    }
}

void TrajectoryPlanner::setCurrentState(const TrajectoryPoint& current_robot_state) {
    current_state_ = current_robot_state;
}

Result<void, TrajectoryPlanner::PlannerError> TrajectoryPlanner::addTargetWaypoint(const TrajectoryPoint& target_waypoint) {
    auto segment_res = interpolator_->createSegment(current_state_, target_waypoint, planning_dt_);

    if (segment_res.isError()) {
        RDT_LOG_ERROR(MODULE_NAME, "Failed to create motion segment.");
        return Result<void, PlannerError>::Failure(segment_res.error());
    }

    trajectory_.addSegment(std::move(segment_res.value()));

    // The end of the new segment becomes the start for the next one
    current_state_ = target_waypoint;

    return Result<void, PlannerError>::Success();
}

Result<void, TrajectoryPlanner::PlannerError> TrajectoryPlanner::overrideTrajectory(const TrajectoryPoint& target_waypoint) {
    RDT_LOG_WARN(MODULE_NAME, "Trajectory override requested. Clearing all buffers.");
    
    // 1. Clear all planned and buffered motion
    trajectory_.clear();
    motion_manager_->reset();

    // 2. Get the very latest physical position from MotionManager's feedback queue
    TrajectoryPoint last_feedback;
    while(motion_manager_->dequeueFeedback(last_feedback)) {
        // Drain the queue to get the last known state
    }
    // TODO: A more robust way to get the last known physical state is needed.
    // For now, we assume the last dequeued point is recent enough.
    setCurrentState(last_feedback);
    
    // 3. Plan a new trajectory from this state
    return addTargetWaypoint(target_waypoint);
}


void TrajectoryPlanner::update() {
    // Determine how many points we can push to the motion manager
    const size_t max_to_push = 50; // Push points in batches to avoid blocking for too long
    size_t free_space = motion_manager_->getCommandQueueSize() > max_to_push
                      ? 0
                      : max_to_push - motion_manager_->getCommandQueueSize();

    if (free_space == 0 || trajectory_.isFinished()) {
        return;
    }

    auto batch = trajectory_.getNextPointBatch(free_space);
    for (const auto& point : batch) {
        motion_manager_->enqueueCommand(point);
    }
}

bool TrajectoryPlanner::isTaskFinished() const {
    return trajectory_.isFinished() && (motion_manager_->getCurrentState() == RTState::Idle);
}

} // namespace RDT
