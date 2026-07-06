// TrajectoryPlanner.h
#pragma once

#include "segments/TrajectoryInterpolator.h"
#include "planner/Trajectory.h"
#include "MotionManager.h"
#include "RobotConfig.h" // For ControllerConfig
#include "RdtProtocol.h"

namespace RDT {

/**
 * @class TrajectoryPlanner
 * @brief The high-level manager for trajectory generation and execution.
 *
 * This class orchestrates the entire planning process. It receives high-level
 * motion commands, uses a `TrajectoryInterpolator` to create `MotionSegment`s,
 * manages them in a `Trajectory` queue, and feeds the resulting points to the
 * `MotionManager`'s buffer.
 * @version 2.2 (Refactored with Configurable Ticks and Preview Logic)
 */
class TrajectoryPlanner {
public:
    using PlannerError = TrajectoryInterpolator::PlannerError;

    /**
     * @brief Constructs the TrajectoryPlanner.
     * @param interpolator A shared pointer to the segment factory.
     * @param motion_manager A shared pointer to the RT motion executor.
     * @param config The controller configuration containing timing parameters.
     */
    TrajectoryPlanner(std::shared_ptr<TrajectoryInterpolator> interpolator,
                      std::shared_ptr<MotionManager> motion_manager,
                      const ControllerConfig& config);
    
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
     * @brief Immediately clears all planned and buffered motion and holds the robot at @p target_waypoint.
     *
     * Stop/pause hold path (REQ-PLAN-06): clears the planner trajectory, calls MotionManager::reset()
     * (blocking queue-clear handshake — the robot is physically held at its last commanded point),
     * then replans a zero-length hold segment to @p target_waypoint. The hold segment is ALWAYS
     * planned in joint space (MotionType::JOINT, command.joint_target), regardless of the motion type
     * carried by @p target_waypoint: production callers (RobotController::stopProgram/pauseProgram)
     * pass the latest feedback point, whose type echoes what the RT loop was executing (HOLD when the
     * RT buffer is empty, SPLINE/CIRC mid-move) and is not plannable as a single segment.
     * @param target_waypoint The live robot point to hold at (caller's latest feedback point).
     * @return A Result indicating success or failure.
     */
    [[nodiscard]] Result<void, PlannerError> overrideTrajectory(const TrajectoryPoint& target_waypoint);

    /**
     * @brief Re-targets a manual jog: drops any unfinished/queued jog motion and replans a fresh
     * segment from the current state to @p target_waypoint. Unlike addTargetWaypoint (which appends
     * a completion-gated segment that the RT loop drains one-at-a-time), this supersedes the previous
     * jog so repeated jog commands never accumulate latency. The core (planner + MotionManager) still
     * owns and executes the jog. `current_state_` is kept fresh by setCurrentState() from the
     * feedback loop, so we plan from the live robot state (unlike overrideTrajectory, which holds
     * position at the point the caller supplies).
     * @param target_waypoint The new jog target.
     * @return A Result indicating success or the reason for a planning failure.
     */
    [[nodiscard]] Result<void, PlannerError> retargetJog(const TrajectoryPoint& target_waypoint);

    /**
     * @brief Plans a contiguous run of motion waypoints as a single continuous trajectory.
     *
     * Unlike calling addTargetWaypoint() per waypoint (where every segment ends with a stop-and-settle
     * gate at the RT layer), this fuses all waypoints into ONE continuous point stream so the robot does
     * not stop/settle at each intermediate waypoint. Two corner behaviors are selected per waypoint by
     * its header.blending_radius (KUKA-style):
     *   - radius == 0 (fine point): the robot still comes to an exact stop at the waypoint, but the next
     *     segment is already queued, so there is no NRT replan gap (look-ahead). Path is exact.
     *   - radius  > 0 (approximate): the corner is rounded by overlapping the deceleration tail of the
     *     incoming segment with the acceleration head of the outgoing one within the blend radius, so the
     *     robot keeps moving through the corner. The waypoint is approached, not reached exactly.
     *
     * The chain always starts from current_state_ (the robot is at rest at the start of a motion run) and
     * always finishes with an exact stop at the final waypoint. Safety: a blend that would exceed the joint
     * velocity ceiling is rejected per-corner and degrades to an exact stop (never an over-speed corner).
     * The RT layer and HAL are unaffected — they consume one longer continuous segment.
     *
     * @param waypoints Ordered motion waypoints (each carries motion_type, targets, speed, blending_radius).
     * @return Success, or the first per-segment PlannerError (e.g. IK_Failed) encountered.
     */
    [[nodiscard]] Result<void, PlannerError> planMotionChain(const std::vector<TrajectoryPoint>& waypoints);

    /**
     * @brief Main update loop for the planner.
     * This non-blocking method should be called periodically from the main application loop.
     * It checks the buffer level of the MotionManager and feeds it new points from the trajectory.
     */
    void update();

    /** @brief Checks if the entire planned trajectory has been sent to the MotionManager. */
    [[nodiscard]] bool isTaskFinished() const;

    /**
     * @brief Generates a trajectory path for visualization without queueing it for execution.
     * This method iterates through a program, plans all motion segments with a coarse time step,
     * and extracts the resulting Cartesian points.
     * @param program The program to visualize.
     * @return A `TrajectoryPathStruct` containing the points for the HMI.
     */
    [[nodiscard]] NetProtocol::TrajectoryPathStruct generatePreviewPath(const NetProtocol::ProgramDataStruct& program);

private:
    /**
     * @brief Attempts to round the corner between an already-fused stream and the next segment without
     * slowing down at the waypoint.
     *
     * Overlap-adds the first K points of @p next_segment onto the last K points of @p fused, then appends
     * the remainder. K = min(K_ramp, K_radius): K_ramp spans the decel ramp of the incoming segment and
     * the accel ramp of the outgoing one (from per-sample joint speed) so a v_peak->0 decel overlaps a
     * 0->v_peak accel and a generous radius gives a full-speed pass-through; K_radius bounds the window to
     * the points within @p radius of the corner so a tight radius keeps the rounded path close to the
     * waypoint (and necessarily carries less speed). If the summed step would exceed @p max_joint_step_deg
     * (a sharp/opposite corner), the window is shrunk and retried; only if even a 1-sample blend is unsafe
     * does it return false so the caller keeps the exact stop.
     *
     * @param[in,out] fused The continuous stream built so far (modified in place on success).
     * @param next_segment The rendered points of the outgoing segment (shares its first point with the corner).
     * @param radius Cartesian zone radius bounding the corner deviation (0 is gated out by the caller).
     * @param[in,out] prev_seg_len Trailing length of @p fused belonging to the incoming segment; updated on success.
     * @param max_joint_step_deg Maximum allowed per-axis change between consecutive points (deg per cycle).
     * @return True if the corner was blended and @p next_segment appended; false if no change was made.
     */
    [[nodiscard]] bool blendCorner(std::vector<TrajectoryPoint>& fused,
                                   const std::vector<TrajectoryPoint>& next_segment,
                                   Millimeters radius,
                                   size_t& prev_seg_len,
                                   double max_joint_step_deg) const;

    std::shared_ptr<TrajectoryInterpolator> interpolator_;
    std::shared_ptr<MotionManager> motion_manager_;
    
    Trajectory trajectory_;
    TrajectoryPoint current_state_;

    const Seconds planning_dt_; // Timestep for segment generation (from config)

    bool dependencies_valid_{true};
    
    static inline const std::string MODULE_NAME = "TrajPlanner";
};

} // namespace RDT