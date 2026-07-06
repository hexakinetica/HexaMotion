// TrajectoryPlanner.cpp
#include "TrajectoryPlanner.h"
#include "LoggingMacros.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace RDT {

TrajectoryPlanner::TrajectoryPlanner(std::shared_ptr<TrajectoryInterpolator> interpolator,
                                     std::shared_ptr<MotionManager> motion_manager,
                                     const ControllerConfig& config)
    : interpolator_(interpolator),
      motion_manager_(motion_manager),
      planning_dt_(config.PlannerTickSec) // Initialize from config
{
    if (!interpolator_ || !motion_manager_) {
        RDT_LOG_CRITICAL(MODULE_NAME, "Interpolator and MotionManager must not be null.");
        dependencies_valid_ = false;
    }
}

void TrajectoryPlanner::setCurrentState(const TrajectoryPoint& current_robot_state) {
    current_state_ = current_robot_state;
}

Result<void, TrajectoryPlanner::PlannerError> TrajectoryPlanner::addTargetWaypoint(const TrajectoryPoint& target_waypoint) {
    if (!dependencies_valid_) {
        return Result<void, PlannerError>::Failure(PlannerError::InvalidArguments);
    }
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
    if (!dependencies_valid_) {
        return Result<void, PlannerError>::Failure(PlannerError::InvalidArguments);
    }
    RDT_LOG_WARN(MODULE_NAME, "Trajectory override requested. Clearing all buffers.");

    // 1. Drop all planned/buffered motion. reset() blocks until the RT loop has cleared its queues
    //    (handshake), so the robot is already physically held at its last commanded point before we
    //    replan — the override cannot race a still-draining pipeline.
    trajectory_.clear();
    motion_manager_->reset();

    // 2. Replan a hold from the live robot state. target_waypoint is the caller's latest feedback
    //    point (RobotController passes getFeedbackTrajectoryPoint()), i.e. where the robot actually
    //    is, so planning a zero-length segment to it holds position there. This uses the same
    //    "plan from the live state" contract as retargetJog(). It replaces the previous manual drain
    //    of MotionManager's feedback queue: reset() had just emptied that queue, so the drain always
    //    fell back to target_waypoint anyway — the drain (and its "more robust way needed" TODO) added
    //    fragility without changing the result.
    //
    //    The hold is ALWAYS planned as a zero-length JOINT segment (REQ-PLAN-06). Feedback points
    //    echo the motion type of whatever the RT loop was executing: MotionType::HOLD when the RT
    //    buffer is empty (MotionManager stamps idle feedback), or SPLINE/CIRC mid-move. None of
    //    those is plannable as a single segment by createSegment() — HOLD/SPLINE are not single-
    //    segment types, and a CIRC whose start, via and target coincide has no arc geometry — so
    //    passing the type through made STOP fail with UnsupportedMotionType whenever the robot was
    //    already standing. Hold-in-place is a joint-space intent: the feedback joints
    //    (command.joint_target = the RT loop's last commanded position) are always valid, need no
    //    Cartesian validity or IK, and a zero-displacement JOINT profile renders deterministically
    //    through the short-segment branch, so the hold replan succeeds for every feedback type.
    TrajectoryPoint hold_target = target_waypoint;
    hold_target.header.motion_type = MotionType::JOINT;
    setCurrentState(hold_target);
    return addTargetWaypoint(hold_target);
}


Result<void, TrajectoryPlanner::PlannerError> TrajectoryPlanner::retargetJog(const TrajectoryPoint& target_waypoint) {
    if (!dependencies_valid_) {
        return Result<void, PlannerError>::Failure(PlannerError::InvalidArguments);
    }

    // Drop the previous (possibly unfinished) jog so jog commands do not pile up behind the
    // MotionManager's per-segment completion gate. reset() blocks until the RT loop has cleared its
    // queues (handshake), so the addTargetWaypoint() below feeds an already-empty pipeline -- no race.
    trajectory_.clear();
    motion_manager_->reset();

    // Plan from the live state. current_state_ is refreshed every cycle by setCurrentState(feedback)
    // in RobotController::processMotionFeedback(), so we start from where the robot actually is.
    return addTargetWaypoint(target_waypoint);
}

Result<void, TrajectoryPlanner::PlannerError>
TrajectoryPlanner::planMotionChain(const std::vector<TrajectoryPoint>& waypoints) {
    if (!dependencies_valid_) {
        return Result<void, PlannerError>::Failure(PlannerError::InvalidArguments);
    }
    if (waypoints.empty()) {
        return Result<void, PlannerError>::Success();
    }

    // 1) Render the chain into segments, chaining from the current state. current_state_ is the
    // robot at rest at the start of the motion run (program start, or after a wait/IO step). The
    // whole chain is built synchronously in one call, so the per-cycle feedback that refreshes
    // current_state_ cannot interleave and corrupt the chain start of a later segment.
    //
    // A maximal contiguous run of SPLINE waypoints is one BLOCK rendered as a single segment
    // (REQ-SPL-01, docs/REQ_motion_spline.md); every other waypoint renders into its own
    // rest-to-rest segment as before. Per-segment metadata (the blend radius of the waypoint
    // ending the segment, and whether the segment is a spline block) replaces the previous 1:1
    // waypoint indexing in the fusion pass below.
    std::vector<std::vector<TrajectoryPoint>> segments;
    std::vector<Millimeters> segment_end_radius;
    std::vector<bool> segment_is_spline;
    segments.reserve(waypoints.size());
    segment_end_radius.reserve(waypoints.size());
    segment_is_spline.reserve(waypoints.size());
    TrajectoryPoint segment_start = current_state_;
    for (size_t i = 0; i < waypoints.size();) {
        if (waypoints[i].header.motion_type == MotionType::SPLINE) {
            size_t block_end = i;
            while (block_end < waypoints.size() &&
                   waypoints[block_end].header.motion_type == MotionType::SPLINE) {
                ++block_end;
            }
            const std::vector<TrajectoryPoint> block(waypoints.begin() + static_cast<std::ptrdiff_t>(i),
                                                     waypoints.begin() + static_cast<std::ptrdiff_t>(block_end));
            auto seg_res = interpolator_->createSplineSegment(segment_start, block, planning_dt_);
            if (seg_res.isError()) {
                RDT_LOG_ERROR(MODULE_NAME,
                    "planMotionChain: failed to render SPLINE block (steps seq {}..{}, error {}).",
                    block.front().header.sequence_index, block.back().header.sequence_index,
                    static_cast<int>(seg_res.error()));
                return Result<void, PlannerError>::Failure(seg_res.error());
            }
            segments.push_back(seg_res.value()->getPoints());
            // Block entry/exit are exact stops in v1 (REQ-SPL-02): the ending radius is forced to
            // zero regardless of what the waypoint carries (blending_radius is ignored on splines).
            segment_end_radius.push_back(0.0_mm);
            segment_is_spline.push_back(true);
            // Chain from the last RENDERED point, not the raw authored waypoint: it carries the
            // solver-derived pose (cartesian_valid set) and the exact endpoint configuration.
            // Both segment factories emit at least one point on success, so back() is safe.
            segment_start = segments.back().back();
            i = block_end;
        } else {
            auto seg_res = interpolator_->createSegment(segment_start, waypoints[i], planning_dt_);
            if (seg_res.isError()) {
                RDT_LOG_ERROR(MODULE_NAME,
                    "planMotionChain: failed to render segment for waypoint seq {} (error {}).",
                    waypoints[i].header.sequence_index, static_cast<int>(seg_res.error()));
                return Result<void, PlannerError>::Failure(seg_res.error());
            }
            segments.push_back(seg_res.value()->getPoints());
            segment_end_radius.push_back(waypoints[i].header.blending_radius);
            segment_is_spline.push_back(false);
            // Chain from the last RENDERED point (see the spline branch above): FK-enriched pose
            // for JOINT endpoints (cartesian_valid set) and the IK branch the path actually
            // evolved to for LIN/CIRC — the raw waypoint carries neither.
            segment_start = segments.back().back();
            ++i;
        }
    }

    // 2) Fuse the segments into one continuous stream, rounding corners whose ending waypoint
    // carries blending_radius > 0. No rendered segment exceeds DEFAULT_JOINT_V_MAX * speed_ratio,
    // so this ceiling is the hard upper bound a blended corner must never cross.
    const double max_joint_step_deg = DEFAULT_JOINT_V_MAX.value() * planning_dt_.value();

    std::vector<TrajectoryPoint> fused;
    size_t prev_seg_len = 0; // trailing points of 'fused' belonging to the most recent segment
    for (size_t i = 0; i < segments.size(); ++i) {
        std::vector<TrajectoryPoint>& segment = segments[i];
        if (segment.empty()) {
            continue; // zero-length move (start == target); nothing to add
        }
        if (fused.empty()) {
            fused = std::move(segment);
            prev_seg_len = fused.size();
            continue;
        }

        // The corner sits at the boundary between fused and this segment; its radius belongs to
        // the segment-ending waypoint before it. A boundary touching a spline block on either side
        // is never blended: the overlap-add would deform the exact curve (REQ-SPL-02).
        const Millimeters radius = (segment_is_spline[i] || segment_is_spline[i - 1])
            ? 0.0_mm : segment_end_radius[i - 1];
        bool blended = false;
        if (radius.value() > 0.0 && segment.size() >= 2 && prev_seg_len >= 2) {
            blended = blendCorner(fused, segment, radius, prev_seg_len, max_joint_step_deg);
        }
        if (!blended) {
            // Fine point: keep the exact stop the interpolator marked on the previous segment's last
            // point, then append this segment without duplicating the shared corner point. The robot
            // still settles here, but the next segment is already queued (no NRT replan gap).
            fused.back().header.is_target_reached_for_this_point = true;
            fused.insert(fused.end(), segment.begin() + 1, segment.end());
            prev_seg_len = segment.size() - 1;
        }
    }

    if (fused.empty()) {
        // Every move was zero-length: nothing to execute, just adopt the final target as the new state.
        current_state_ = waypoints.back();
        RDT_LOG_INFO(MODULE_NAME, "planMotionChain: chain of {} waypoint(s) produced no motion.",
            waypoints.size());
        return Result<void, PlannerError>::Success();
    }

    // 3) The chain always ends with an exact stop at the final waypoint.
    fused.back().header.is_target_reached_for_this_point = true;

    const size_t fused_size = fused.size();
    trajectory_.addSegment(std::make_unique<MotionSegment>(std::move(fused), nullptr));
    current_state_ = waypoints.back();

    RDT_LOG_INFO(MODULE_NAME, "planMotionChain: fused {} waypoint(s) into {} continuous points.",
        waypoints.size(), fused_size);
    return Result<void, PlannerError>::Success();
}

bool TrajectoryPlanner::blendCorner(std::vector<TrajectoryPoint>& fused,
                                    const std::vector<TrajectoryPoint>& next_segment,
                                    Millimeters radius,
                                    size_t& prev_seg_len,
                                    double max_joint_step_deg) const {
    // Max-axis joint step (deg) between two points -- a proxy for joint speed * dt.
    const auto jointStep = [](const TrajectoryPoint& a, const TrajectoryPoint& b) -> double {
        double m = 0.0;
        for (std::size_t ax = 0; ax < ROBOT_AXES_COUNT; ++ax) {
            const AxisId axis = static_cast<AxisId>(ax);
            const double d = std::abs(a.command.joint_target[axis].position.value()
                                    - b.command.joint_target[axis].position.value());
            m = std::max(m, d);
        }
        return m;
    };
    const auto cartDist = [](const CartPose& a, const CartPose& b) -> double {
        const double dx = a.x.value() - b.x.value();
        const double dy = a.y.value() - b.y.value();
        const double dz = a.z.value() - b.z.value();
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    // The blend window K trades corner speed against corner size and is the min of two limits:
    //
    //  - K_ramp: overlapping the FULL deceleration ramp of the incoming move with the FULL acceleration
    //    ramp of the outgoing one keeps the summed corner speed ~v_peak (pass-through, no slowdown).
    //    This is the maximum useful overlap; extending past it (into cruise) would sum two full speeds
    //    and overshoot the joint-speed limit.
    //  - K_radius: the number of points within the Cartesian blend @p radius of the corner. This bounds
    //    how far the rounded path may deviate from the waypoint (the operator-set zone radius).
    //
    // K = min(K_ramp, K_radius): a generous radius gives a full-speed fly-by; a tight radius keeps the
    // corner close to the waypoint and therefore carries less speed (standard zone-radius behavior).
    const size_t in_begin = fused.size() - prev_seg_len; // first index of the incoming segment in fused

    double peak_in = 0.0;
    for (size_t j = in_begin + 1; j < fused.size(); ++j) {
        peak_in = std::max(peak_in, jointStep(fused[j], fused[j - 1]));
    }
    size_t kb = 0; // trailing deceleration-ramp samples of the incoming segment
    for (size_t j = fused.size() - 1; j > in_begin; --j) {
        if (jointStep(fused[j], fused[j - 1]) < peak_in * (1.0 - 1e-3)) {
            ++kb;
        } else {
            break; // reached the cruise plateau (or the profile peak)
        }
    }

    double peak_out = 0.0;
    for (size_t j = 1; j < next_segment.size(); ++j) {
        peak_out = std::max(peak_out, jointStep(next_segment[j], next_segment[j - 1]));
    }
    size_t kf = 0; // leading acceleration-ramp samples of the outgoing segment
    for (size_t j = 1; j < next_segment.size(); ++j) {
        if (jointStep(next_segment[j], next_segment[j - 1]) < peak_out * (1.0 - 1e-3)) {
            ++kf;
        } else {
            break;
        }
    }
    const size_t k_ramp = std::min(kb, kf);

    const double r = radius.value();
    const CartPose corner = fused.back().command.cartesian_target;
    size_t k_back_radius = 0; // trailing incoming points within the zone radius of the corner
    for (size_t s = 1; s < prev_seg_len && s < fused.size(); ++s) {
        if (cartDist(fused[fused.size() - 1 - s].command.cartesian_target, corner) <= r) {
            k_back_radius = s;
        } else {
            break;
        }
    }
    size_t k_fwd_radius = 0; // leading outgoing points within the zone radius of the corner
    for (size_t s = 1; s < next_segment.size(); ++s) {
        if (cartDist(next_segment[s].command.cartesian_target, corner) <= r) {
            k_fwd_radius = s;
        } else {
            break;
        }
    }
    const size_t k_radius = std::min(k_back_radius, k_fwd_radius);

    size_t k = std::min(k_ramp, k_radius);
    k = std::min(k, prev_seg_len - 1);
    k = std::min(k, next_segment.size() - 1);
    if (k < 1) {
        return false; // radius/ramps too short to overlap -> caller keeps the exact stop
    }

    const AxisSet& seg_head = next_segment[0].command.joint_target;
    const double step_ceiling = max_joint_step_deg * 1.05; // small tolerance for sampling discretization

    const auto within_limit = [&](const std::array<Degrees, ROBOT_AXES_COUNT>& p,
                                  const std::array<Degrees, ROBOT_AXES_COUNT>& q) -> bool {
        for (std::size_t ax = 0; ax < ROBOT_AXES_COUNT; ++ax) {
            if ((p[ax] - q[ax]).abs().value() > step_ceiling) {
                return false;
            }
        }
        return true;
    };

    // Build the overlap-added window, velocity-check it, and commit. The window covers
    // fused[base .. base+k-1] paired with next_segment[0 .. k-1] via offset next_segment[s] -
    // next_segment[0]; the window start is unchanged and its end becomes next_segment[k-1], so the
    // appended remainder continues seamlessly. A sharp/opposite corner whose summed step would exceed
    // the joint-speed ceiling is not rejected outright: the window is shrunk and retried (less
    // rounding, slight slowdown), degrading to an exact stop only if even a 1-sample blend is unsafe.
    while (k >= 1) {
        const size_t base = fused.size() - k;
        std::vector<std::array<Degrees, ROBOT_AXES_COUNT>> candidate(k);
        for (size_t s = 0; s < k; ++s) {
            for (std::size_t ax = 0; ax < ROBOT_AXES_COUNT; ++ax) {
                const AxisId axis = static_cast<AxisId>(ax);
                const Degrees offset =
                    next_segment[s].command.joint_target[axis].position - seg_head[axis].position;
                candidate[s][ax] = fused[base + s].command.joint_target[axis].position + offset;
            }
        }

        bool ok = true;
        if (base > 0) {
            ok = within_limit(fused[base - 1].command.joint_target.ToPositionArray(), candidate[0]);
        }
        for (size_t s = 1; ok && s < k; ++s) {
            ok = within_limit(candidate[s - 1], candidate[s]);
        }

        if (ok) {
            const uint32_t entering_seq = next_segment[0].header.sequence_index;
            for (size_t s = 0; s < k; ++s) {
                (void)fused[base + s].command.joint_target.SetFromPositionArray(candidate[s]);
                fused[base + s].header.is_target_reached_for_this_point = false;
                fused[base + s].header.use_blending = true;
                fused[base + s].header.sequence_index = entering_seq;
            }
            fused.insert(fused.end(), next_segment.begin() + static_cast<std::ptrdiff_t>(k),
                         next_segment.end());
            prev_seg_len = next_segment.size() - k;
            return true;
        }
        k /= 2; // window too aggressive for this corner -- reduce and retry
    }
    return false;
}

void TrajectoryPlanner::update() {
    if (!dependencies_valid_) {
        return;
    }
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
    if (!dependencies_valid_) {
        return true;
    }
    return trajectory_.isFinished() && (motion_manager_->getCurrentState() == RTState::Idle);
}

NetProtocol::TrajectoryPathStruct TrajectoryPlanner::generatePreviewPath(const NetProtocol::ProgramDataStruct& program) {
    NetProtocol::TrajectoryPathStruct path;
    if (!dependencies_valid_ || program.steps.empty() || !interpolator_) {
        return path;
    }

    // Use a moderate time step for preview generation (~60 Hz) to better capture curved paths
    const Seconds preview_dt = Seconds(1.0 / 60.0);

    // Use the current robot state as the absolute starting point for the preview
    TrajectoryPoint start_state = current_state_;
    // Fall back to the feedback pose when the command pose is invalid. This is the ONE place the
    // magic-zero test survives: RobotFeedbackFrame carries no validity flag, and an all-zero
    // cartesian_actual only occurs before the first FK enrichment (startup), so validity is
    // re-derived explicitly at this boundary. Everywhere downstream validity travels as
    // command.cartesian_valid.
    if (!start_state.command.cartesian_valid) {
        const CartPose& fb = start_state.feedback.cartesian_actual;
        start_state.command.cartesian_target = fb;
        start_state.command.cartesian_valid =
            !(fb.x.value() == 0.0 && fb.y.value() == 0.0 && fb.z.value() == 0.0);
    }

    bool has_last_valid_point = false;
    NetProtocol::TrajectoryPointStruct last_valid_point{};

    // Consumes one rendered segment into the preview path: downsampled points (invalid-pose points
    // are skipped — the FK-failed fallback), the end waypoint marker, and — for spline blocks — a
    // marker at every authored inner point (detected by the sequence_index transition the
    // interpolator stamps). continuation_target keeps the chain start correct when the segment
    // rendered empty.
    const auto consumeSegment = [&](const std::vector<TrajectoryPoint>& segment_points,
                                    const TrajectoryPoint& continuation_target) {
        // Downsample points to avoid overloading the network/UI while preserving curvature
        const size_t max_points = 400;
        const size_t stride = std::max<size_t>(1, segment_points.size() / max_points);
        for (size_t i = 0; i < segment_points.size(); i += stride) {
            const auto& point = segment_points[i];
            if (!point.command.cartesian_valid) {
                if (has_last_valid_point) {
                    path.points.push_back(last_valid_point);
                }
                continue;
            }
            last_valid_point = {
                point.command.cartesian_target.x,
                point.command.cartesian_target.y,
                point.command.cartesian_target.z
            };
            has_last_valid_point = true;
            path.points.push_back(last_valid_point);
        }
        // Inner waypoint markers of a spline block: the rendered stream advances sequence_index as
        // the curve passes each authored point; mark the last rendered point of every span.
        for (size_t i = 0; i + 1 < segment_points.size(); ++i) {
            if (segment_points[i].header.sequence_index != segment_points[i + 1].header.sequence_index &&
                segment_points[i].command.cartesian_valid) {
                path.waypoints.push_back({
                    segment_points[i].command.cartesian_target.x,
                    segment_points[i].command.cartesian_target.y,
                    segment_points[i].command.cartesian_target.z
                });
            }
        }
        // Ensure the last point of the segment is included
        if (!segment_points.empty()) {
            const auto& last = segment_points.back();
            NetProtocol::TrajectoryPointStruct last_point{};
            // Explicit known/unknown tracking: comparing last_point against a default-constructed
            // struct would silently drop a legitimate endpoint AT the origin (the same magic-zero
            // smell the cartesian_valid flag removed).
            bool last_point_known = false;
            if (last.command.cartesian_valid) {
                last_point = {
                    last.command.cartesian_target.x,
                    last.command.cartesian_target.y,
                    last.command.cartesian_target.z
                };
                last_valid_point = last_point;
                has_last_valid_point = true;
                last_point_known = true;
            } else if (has_last_valid_point) {
                last_point = last_valid_point;
                last_point_known = true;
            }

            if (last_point_known) {
                path.points.push_back(last_point);

                // Add the actual calculated end-point as the waypoint marker
                // This is more accurate than using the raw target for MoveJ since it reflects the actual FK result
                path.waypoints.push_back(last_point);
            }

            // The start of the next segment is the end of this one (use full state for continuity)
            start_state = last;
        } else {
            // If the segment was empty (e.g., zero duration because start == target),
            // we must still update the start state to the target waypoint to ensure continuity.
            start_state = continuation_target;
            if (continuation_target.command.cartesian_valid) {
                // Ensure at least one point is added so the first waypoint is visible in preview
                last_valid_point = {
                    continuation_target.command.cartesian_target.x,
                    continuation_target.command.cartesian_target.y,
                    continuation_target.command.cartesian_target.z
                };
                has_last_valid_point = true;
                path.points.push_back(last_valid_point);
                path.waypoints.push_back(last_valid_point);
            } else if (has_last_valid_point) {
                path.points.push_back(last_valid_point);
            }
        }
    };

    // Builds the internal waypoint for one motion program step (shared by the single-step and
    // spline-block branches below).
    const auto waypointFromStep = [](const NetProtocol::ProgramStepStruct& step, uint32_t step_index) {
        TrajectoryPoint wp;
        if (step.type == NetProtocol::StepType::MoveJ) {
            wp.header.motion_type = MotionType::JOINT;
        } else if (step.type == NetProtocol::StepType::MoveL) {
            wp.header.motion_type = MotionType::LIN;
        } else if (step.type == NetProtocol::StepType::MoveC) {
            wp.header.motion_type = MotionType::CIRC;
            wp.command.cartesian_via = step.cartesian_via;
        } else { // NetProtocol::StepType::MoveS
            wp.header.motion_type = MotionType::SPLINE;
        }
        wp.header.sequence_index = step_index;
        wp.command.joint_target = step.joint_target;
        wp.command.cartesian_target = step.cartesian_target;
        // Validity derives from the motion TYPE at this wire boundary: LIN/CIRC/SPLINE steps author
        // a real Cartesian pose (mandatory on the wire); for JOINT the joints are the command and
        // the Cartesian preview is derived by FK at render time (one source of truth: the solver).
        wp.command.cartesian_valid = (wp.header.motion_type != MotionType::JOINT);
        wp.command.speed_ratio = step.speed_ratio / 100.0;
        return wp;
    };

    const auto isMotionStep = [](NetProtocol::StepType type) {
        return type == NetProtocol::StepType::MoveJ || type == NetProtocol::StepType::MoveL ||
               type == NetProtocol::StepType::MoveC || type == NetProtocol::StepType::MoveS;
    };

    for (size_t idx = 0; idx < program.steps.size();) {
        const auto& step = program.steps[idx];
        // We only visualize motion steps
        if (!isMotionStep(step.type)) {
            ++idx;
            continue;
        }

        if (step.type == NetProtocol::StepType::MoveS) {
            // A contiguous MoveS run previews as ONE spline block, exactly as it executes
            // (REQ-SPL-01) — previewing it per-point would draw a path the robot never drives.
            std::vector<TrajectoryPoint> block;
            while (idx < program.steps.size() && program.steps[idx].type == NetProtocol::StepType::MoveS) {
                block.push_back(waypointFromStep(program.steps[idx], static_cast<uint32_t>(idx)));
                ++idx;
            }
            auto segment_res = interpolator_->createSplineSegment(start_state, block, preview_dt);
            if (segment_res.isError()) {
                RDT_LOG_WARN(MODULE_NAME, "Failed to generate SPLINE block for preview. Stopping preview generation.");
                break;
            }
            consumeSegment(segment_res.value()->getPoints(), block.back());
        } else {
            const TrajectoryPoint target_waypoint = waypointFromStep(step, static_cast<uint32_t>(idx));
            ++idx;
            auto segment_res = interpolator_->createSegment(start_state, target_waypoint, preview_dt);
            if (segment_res.isError()) {
                RDT_LOG_WARN(MODULE_NAME, "Failed to generate segment for preview. Stopping preview generation.");
                break;
            }
            consumeSegment(segment_res.value()->getPoints(), target_waypoint);
        }
    }
    return path;
}

} // namespace RDT
