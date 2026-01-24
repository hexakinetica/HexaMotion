// TrajectoryInterpolator.h
#ifndef TRAJECTORY_INTERPOLATOR_H
#define TRAJECTORY_INTERPOLATOR_H

#pragma once

#include "MotionSegment.h"
#include "kinematic_solver/KinematicSolver.h"
#include "result.h"
#include "ErrorCode.h"
#include <memory>

namespace RDT {

// Default kinematic limits for profile generation if not overridden.
constexpr DegreesPerSecond DEFAULT_JOINT_V_MAX = 90.0_deg_s;      // ~1.57 rad/s
constexpr DegreesPerSecondSq DEFAULT_JOINT_A_MAX = 180.0_deg_s2;   // ~3.14 rad/s2
constexpr MillimetersPerSecond DEFAULT_CART_V_MAX = 250.0_mm_s;    // 0.25 m/s
constexpr MillimetersPerSecondSq DEFAULT_CART_A_MAX = 500.0_mm_s2;  // 0.5 m/s^2

/**
 * @class TrajectoryInterpolator
 * @brief A factory for creating fully calculated MotionSegment objects.
 *
 * This class takes a start and target point, creates the appropriate MotionProfile,
 * and then "renders" that profile into a complete MotionSegment containing all
 * intermediate points. For LIN segments, it performs all necessary IK calculations
 * during this creation phase.
 * @version 2.0 (Refactored)
 */
class TrajectoryInterpolator {
public:
    /**
     * @brief Constructs the TrajectoryInterpolator.
     * @param solver A shared pointer to the kinematic solver, required for LIN movements.
     */
    explicit TrajectoryInterpolator(std::shared_ptr<KinematicSolver> solver);
    ~TrajectoryInterpolator() = default;

    // This class is stateless (a factory), so it can be copied/moved.
    TrajectoryInterpolator(const TrajectoryInterpolator&) = default;
    TrajectoryInterpolator& operator=(const TrajectoryInterpolator&) = default;
    TrajectoryInterpolator(TrajectoryInterpolator&&) = default;
    TrajectoryInterpolator& operator=(TrajectoryInterpolator&&) = default;
    
    /**
     * @enum PlannerError
     * @brief Defines errors that can occur during segment generation.
     */
    enum class PlannerError {
        Ok,
        IK_Failed,
        FK_Failed,
        UnsupportedMotionType,
        InvalidArguments
    };

    /**
     * @brief Creates a complete MotionSegment.
     * This is a computationally intensive operation that performs all calculations for a segment upfront.
     * @param start The starting state of the segment.
     * @param target The target state of the segment (defines motion type, speed, etc.).
     * @param dt The time step for generating intermediate points.
     * @return A Result containing a unique_ptr to the new MotionSegment on success, or a PlannerError on failure.
     */
    [[nodiscard]] Result<std::unique_ptr<MotionSegment>, PlannerError> createSegment(
        const TrajectoryPoint& start,
        const TrajectoryPoint& target,
        Seconds dt
    );

private:
    std::shared_ptr<KinematicSolver> solver_;
    static inline const std::string MODULE_NAME = "Interpolator";
};

} // namespace RDT
#endif // TRAJECTORY_INTERPOLATOR_H