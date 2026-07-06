// TrajectoryInterpolator.h
#pragma once

#include "MotionSegment.h"
#include "kinematic_solver/KinematicSolver.h"
#include "result.h"
#include "ErrorCode.h"
#include <memory>

namespace RDT {

// Default kinematic limits for profile generation if not overridden. These are the CEILING that
// the upper-panel SPEED% selector scales: a segment runs at DEFAULT_*_V_MAX * speed_ratio (0..1),
// so 100% == these numbers and the planner cannot exceed them.
//
// JOINT dynamics are aligned with the axis speed the MKS backend is actually commanded
// (RobotConfig::default_vel_deg_s = 360 deg/s at SPEED%=100), REQ_axis_dynamics_defaults: the
// previous 720/2160 pair made PTP in simulation fly at up to twice what the real drive is even
// asked to do, which read as "PTP is unrealistically fast next to LIN". No per-axis passport
// limits exist yet; when they do, they belong in the robot config, not here.
// NOTE: shared by ALL planned motion (jog + programs) but only governs the streamed profile
// (internal SimDriver / UDP stream). The MKS backend is point-to-point and runs at the Motion
// command's target_vel (default_vel_deg_s * SPEED%), not at this planner ceiling — keeping the
// two ceilings equal is exactly what makes SIM pacing match the real axis command.
constexpr DegreesPerSecond DEFAULT_JOINT_V_MAX = 360.0_deg_s;      // ~6.28 rad/s (= default_vel_deg_s)
constexpr DegreesPerSecondSq DEFAULT_JOINT_A_MAX = 720.0_deg_s2;   // reaches v_max in 0.5 s
constexpr MillimetersPerSecond DEFAULT_CART_V_MAX = 1000.0_mm_s;   // 1.0 m/s (was 250)
constexpr MillimetersPerSecondSq DEFAULT_CART_A_MAX = 2000.0_mm_s2; // 2.0 m/s^2 (was 500)

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
        InvalidArguments,
        InvalidPathGeometry,     ///< CIRC/SPLINE points do not define a traversable path
                                 ///< (REQ-CIRC-04 / REQ-SPL-09).
        ExcessiveJointVelocity   ///< A rendered SPLINE point demands a per-cycle joint step above
                                 ///< DEFAULT_JOINT_V_MAX * dt — likely near a singularity (REQ-SPL-08).
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

    /**
     * @brief Creates one MotionSegment for a whole SPLINE block (docs/REQ_motion_spline.md).
     *
     * The block is the maximal contiguous run of MotionType::SPLINE waypoints gathered by the
     * planner (REQ-SPL-01). The single-target createSegment() cannot express N points, so the
     * spline has its own entry using the same rendering conventions: IK per point with an evolving
     * seed, endpoint IK re-seeded from the rendered path, segment_target stamped on every point.
     * Each rendered point carries the header (sequence_index) of the authored point ending its
     * span, so the HMI executing line advances through the block. The block runs at the FIRST
     * waypoint's speed/acceleration ratios (REQ-SPL-05).
     *
     * @param start The chain state at block entry.
     * @param block The authored spline waypoints, in program order (must not be empty).
     * @param dt    The time step for generating intermediate points.
     * @return The rendered segment, or a typed PlannerError (InvalidPathGeometry on degenerate
     *         authored geometry, ExcessiveJointVelocity when the joint-step guard fires).
     */
    [[nodiscard]] Result<std::unique_ptr<MotionSegment>, PlannerError> createSplineSegment(
        const TrajectoryPoint& start,
        const std::vector<TrajectoryPoint>& block,
        Seconds dt
    );

private:
    std::shared_ptr<KinematicSolver> solver_;
    // False when constructed without a solver. createSegment() then returns InvalidArguments instead
    // of dereferencing a null solver. Set instead of throwing from the constructor (mandate).
    bool dependencies_valid_ = false;
    static inline const std::string MODULE_NAME = "Interpolator";
};

} // namespace RDT