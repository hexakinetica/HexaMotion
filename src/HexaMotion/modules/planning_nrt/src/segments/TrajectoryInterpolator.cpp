// TrajectoryInterpolator.cpp
#include "TrajectoryInterpolator.h"
#include "LoggingMacros.h"

namespace RDT {

TrajectoryInterpolator::TrajectoryInterpolator(std::shared_ptr<KinematicSolver> solver)
    : solver_(std::move(solver)) {
    // Mandate: no exceptions. A null solver leaves the interpolator invalid; createSegment() then
    // returns InvalidArguments so the planner surfaces a typed error instead of crashing.
    if (!solver_) {
        RDT_LOG_CRITICAL(MODULE_NAME,
            "KinematicSolver is null. Interpolator is INVALID; no motion segments can be created "
            "until it is reconstructed with a valid solver.");
        dependencies_valid_ = false;
    } else {
        dependencies_valid_ = true;
    }
}

Result<std::unique_ptr<MotionSegment>, TrajectoryInterpolator::PlannerError>
TrajectoryInterpolator::createSegment(const TrajectoryPoint& start, const TrajectoryPoint& target, Seconds dt) {

    if (!dependencies_valid_ || !solver_) {
        RDT_LOG_ERROR(MODULE_NAME, "createSegment called on an interpolator with no kinematic solver.");
        return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::InvalidArguments);
    }

    // Fail-closed start-pose gate for Cartesian motion: LIN/CIRC build their path FROM the start
    // pose, so an invalid one (FK never succeeded on this state) would plan a real move through the
    // workspace from a garbage point. Refuse with a typed error instead; the robot holds position
    // and the caller reports the fault. JOINT/PTP plan in joint space and do not need it.
    if ((target.header.motion_type == MotionType::LIN ||
         target.header.motion_type == MotionType::CIRC) &&
        !start.command.cartesian_valid) {
        RDT_LOG_ERROR(MODULE_NAME,
            "Cartesian segment rejected for waypoint seq {}: the start state carries no valid "
            "Cartesian pose (FK enrichment has not produced one). Nothing is planned; the robot "
            "holds position. Check the kinematic solver / feedback path and retry.",
            target.header.sequence_index);
        return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::InvalidArguments);
    }

    // 1. Create the appropriate motion profile. The profile constructors do not throw (they only run
    //    trapezoidal / quaternion math), so no exception boundary is needed here.
    std::shared_ptr<MotionProfile> profile;
    const MotionType motion_type = target.header.motion_type;
    const double v_factor = std::max(0.01, std::min(1.0, target.command.speed_ratio));
    const double a_factor = std::max(0.01, std::min(1.0, target.command.acceleration_ratio));

    if (motion_type == MotionType::JOINT || motion_type == MotionType::PTP) {
        profile = std::make_shared<JointMotionProfile>(
            start.command.joint_target, target.command.joint_target,
            DEFAULT_JOINT_V_MAX * v_factor, DEFAULT_JOINT_A_MAX * a_factor);
    } else if (motion_type == MotionType::LIN) {
        profile = std::make_shared<LinMotionProfile>(
            start.command.cartesian_target, target.command.cartesian_target,
            DEFAULT_CART_V_MAX * v_factor, DEFAULT_CART_A_MAX * a_factor);
    } else if (motion_type == MotionType::CIRC) {
        auto circ_profile = std::make_shared<CircMotionProfile>(
            start.command.cartesian_target, target.command.cartesian_via, target.command.cartesian_target,
            DEFAULT_CART_V_MAX * v_factor, DEFAULT_CART_A_MAX * a_factor);
        if (!circ_profile->isValid()) {
            // Fail-closed (REQ-CIRC-04): the three points do not define a traversable arc. The
            // segment is not planned, previously planned motion is untouched, and the caller
            // (program fault path) reports the step to the operator. Safe to continue after the
            // program is corrected.
            RDT_LOG_ERROR(MODULE_NAME,
                "CIRC segment rejected for waypoint seq {}: {}. start=({}, {}, {}) via=({}, {}, {}) "
                "target=({}, {}, {}) [mm]. The arc is NOT planned; fix the program points and restart.",
                target.header.sequence_index,
                CircMotionProfile::geometryErrorName(circ_profile->geometryError()),
                start.command.cartesian_target.x.value(), start.command.cartesian_target.y.value(),
                start.command.cartesian_target.z.value(),
                target.command.cartesian_via.x.value(), target.command.cartesian_via.y.value(),
                target.command.cartesian_via.z.value(),
                target.command.cartesian_target.x.value(), target.command.cartesian_target.y.value(),
                target.command.cartesian_target.z.value());
            return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::InvalidPathGeometry);
        }
        profile = circ_profile;
    } else {
        RDT_LOG_ERROR(MODULE_NAME, "Unsupported motion type for segment creation: {}", static_cast<int>(motion_type));
        return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::UnsupportedMotionType);
    }

    // 2. "Render" the profile into a vector of points
    std::vector<TrajectoryPoint> points;
    Seconds duration = profile->getDuration();
    AxisSet last_ik_seed = start.command.joint_target;
    // LIN and CIRC are both Cartesian-path profiles rendered by IK per point; JOINT/PTP interpolate
    // joints directly.
    const bool is_cartesian = (profile->getMotionType() == MotionType::LIN ||
                               profile->getMotionType() == MotionType::CIRC);
    const std::array<Degrees, ROBOT_AXES_COUNT> start_angles = start.command.joint_target.ToPositionArray();

    // Strict angular target for the end of the segment. For LIN/CIRC it is computed by IK of the
    // target pose, but ONLY after the path has been rendered, so the endpoint IK is seeded with the
    // evolved near-endpoint configuration (last_ik_seed) rather than the far start seed. The KDL NR
    // solver converges to the solution nearest the seed, so seeding with the start pose could pick a
    // different IK branch than the streamed path (a jump at the segment end, and a segment_target
    // that disagrees with the last streamed point) or fail outright on a large reachable move.
    // For JOINT/PTP the target joints are given directly, so no IK seeding is involved.
    std::array<Degrees, ROBOT_AXES_COUNT> final_target_angles{};

    if (duration.value() < dt.value()) { // Segment is shorter than one time step, or zero length
        // Ensure cartesian target is populated for preview, especially for JOINT/PTP moves.
        TrajectoryPoint preview_target = target;
        if (is_cartesian) {
            // Single endpoint point: only the start seed is available (no path to evolve along).
            auto ik_res = solver_->solveIK(target.command.cartesian_target, last_ik_seed);
            if (ik_res.isError()) {
                RDT_LOG_ERROR(MODULE_NAME, "IK failed for final target cartesian pose during LIN/CIRC segment generation");
                return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::IK_Failed);
            }
            final_target_angles = ik_res.value().ToPositionArray();
            (void)preview_target.command.joint_target.SetFromPositionArray(final_target_angles);
            // The endpoint pose is the authored Cartesian target the IK just solved: stamp validity
            // here (the producer knows) instead of relying on the incoming waypoint's flag.
            preview_target.command.cartesian_valid = true;
        } else {
            final_target_angles = target.command.joint_target.ToPositionArray();
            if ((profile->getMotionType() == MotionType::JOINT || profile->getMotionType() == MotionType::PTP) &&
                !preview_target.command.cartesian_valid) {
                CartPose fk_res{};
                if (solver_->solveFK(preview_target.command.joint_target, fk_res)) {
                    preview_target.command.cartesian_target = fk_res;
                    preview_target.command.cartesian_valid = true;
                }
            }
        }
        preview_target.segment_target.start_angles = start_angles;
        preview_target.segment_target.target_angles = final_target_angles;
        preview_target.segment_target.motion_duration = duration;
        preview_target.segment_target.speed_ratio = target.command.speed_ratio;
        points.push_back(preview_target); // Always add the target point, even if duration is zero, to ensure execution continuity.
    } else {
        // Render the intermediate points. For LIN/CIRC the IK seed evolves along the path so it
        // tracks one continuous branch.
        for (Seconds t = dt; t < duration; t += dt) {
            TrajectoryPoint point;
            point.header = target.header;

            if (is_cartesian) {
                auto cart_res = profile->interpolateCartesian(t);
                if (cart_res.isError()) {
                    RDT_LOG_ERROR(MODULE_NAME, "Cartesian interpolation failed during LIN/CIRC segment generation at t={}", t.toString());
                    return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::InvalidArguments);
                }
                const CartPose cart_pose = cart_res.value();
                auto ik_res = solver_->solveIK(cart_pose, last_ik_seed);
                if (ik_res.isError()) {
                    RDT_LOG_ERROR(MODULE_NAME, "IK failed during LIN/CIRC segment generation at t={}", t.toString());
                    return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::IK_Failed);
                }
                point.command.joint_target = ik_res.value();
                point.command.cartesian_target = cart_pose; // Store Cartesian for preview
                point.command.cartesian_valid = true;
                last_ik_seed = ik_res.value();
            } else { // JOINT or PTP
                auto joint_res = profile->interpolateJoints(t);
                if (joint_res.isError()) {
                    RDT_LOG_ERROR(MODULE_NAME, "Joint interpolation failed during JOINT/PTP segment generation at t={}", t.toString());
                    return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::InvalidArguments);
                }
                point.command.joint_target = joint_res.value();
                // Compute FK for preview. If FK fails the cartesian stays flagged invalid (and the
                // preview filter skips the point); joints remain valid.
                CartPose fk_res{};
                if (solver_->solveFK(point.command.joint_target, fk_res)) {
                    point.command.cartesian_target = fk_res;
                    point.command.cartesian_valid = true;
                }
            }
            points.push_back(point);
        }

        // Endpoint. For LIN/CIRC, seed the IK with the evolved near-endpoint configuration for
        // continuity with the rendered path (see the note above), NOT the far start seed.
        TrajectoryPoint final_point = target;

        if (is_cartesian) {
            auto ik_res = solver_->solveIK(target.command.cartesian_target, last_ik_seed);
            if (ik_res.isError()) {
                RDT_LOG_ERROR(MODULE_NAME, "IK failed for final target cartesian pose during LIN/CIRC segment generation");
                return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::IK_Failed);
            }
            final_target_angles = ik_res.value().ToPositionArray();
            (void)final_point.command.joint_target.SetFromPositionArray(final_target_angles);
            // Same producer-side stamp as the short-segment branch: the endpoint pose is real.
            final_point.command.cartesian_valid = true;
        } else {
            final_target_angles = target.command.joint_target.ToPositionArray();
            CartPose fk_res{};
            if (solver_->solveFK(final_point.command.joint_target, fk_res)) {
                final_point.command.cartesian_target = fk_res;
                final_point.command.cartesian_valid = true;
            }
        }
        points.push_back(final_point); // Always add the final target point to ensure precision

        // Stamp the segment goal on every point now that the (evolved-seed) endpoint is known. Doing
        // it in one pass keeps segment_target.target_angles identical to the final rendered joints.
        for (auto& p : points) {
            p.segment_target.start_angles = start_angles;
            p.segment_target.target_angles = final_target_angles;
            p.segment_target.motion_duration = duration;
            p.segment_target.speed_ratio = target.command.speed_ratio;
        }
    }
    
    // Mark the very last point as "reached"
    if (!points.empty()) {
        points.back().header.is_target_reached_for_this_point = true;
    }

    RDT_LOG_INFO(MODULE_NAME, "Created MotionSegment of type {} with {} points. Duration: {}",
        static_cast<int>(profile->getMotionType()), points.size(), duration.toString());
    
    return Result<std::unique_ptr<MotionSegment>, PlannerError>::Success(
        std::make_unique<MotionSegment>(std::move(points), profile)
    );
}

Result<std::unique_ptr<MotionSegment>, TrajectoryInterpolator::PlannerError>
TrajectoryInterpolator::createSplineSegment(const TrajectoryPoint& start,
                                            const std::vector<TrajectoryPoint>& block,
                                            Seconds dt) {
    if (!dependencies_valid_ || !solver_) {
        RDT_LOG_ERROR(MODULE_NAME, "createSplineSegment called on an interpolator with no kinematic solver.");
        return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::InvalidArguments);
    }
    if (block.empty()) {
        RDT_LOG_ERROR(MODULE_NAME, "createSplineSegment called with an empty spline block.");
        return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::InvalidArguments);
    }
    // Same fail-closed start-pose gate as createSegment: the spline curve starts FROM the start
    // pose, so planning from an invalid one would drive the arm through the workspace from a
    // garbage point. Refuse with a typed error; the robot holds position.
    if (!start.command.cartesian_valid) {
        RDT_LOG_ERROR(MODULE_NAME,
            "SPLINE block rejected (steps seq {}..{}): the start state carries no valid Cartesian "
            "pose (FK enrichment has not produced one). Nothing is planned; the robot holds "
            "position. Check the kinematic solver / feedback path and retry.",
            block.front().header.sequence_index, block.back().header.sequence_index);
        return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::InvalidArguments);
    }

    // The block runs at the FIRST waypoint's ratios (REQ-SPL-05); the editor warns on mixed speeds.
    const double v_factor = std::max(0.01, std::min(1.0, block.front().command.speed_ratio));
    const double a_factor = std::max(0.01, std::min(1.0, block.front().command.acceleration_ratio));

    std::vector<CartPose> targets;
    targets.reserve(block.size());
    for (const TrajectoryPoint& wp : block) {
        targets.push_back(wp.command.cartesian_target);
    }

    auto profile = std::make_shared<SplineMotionProfile>(
        start.command.cartesian_target, targets,
        DEFAULT_CART_V_MAX * v_factor, DEFAULT_CART_A_MAX * a_factor);
    if (!profile->isValid()) {
        // Fail-closed (REQ-SPL-09): the authored points do not define a traversable curve. Nothing
        // is planned, previously planned motion is untouched, the program faults with an
        // operator-visible reason. Safe to re-teach the points and restart.
        RDT_LOG_ERROR(MODULE_NAME,
            "SPLINE block rejected (steps seq {}..{}, {} point(s)): {}. The block is NOT planned; "
            "fix the program points and restart.",
            block.front().header.sequence_index, block.back().header.sequence_index, block.size(),
            SplineMotionProfile::geometryErrorName(profile->geometryError()));
        return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::InvalidPathGeometry);
    }

    const Seconds duration = profile->getDuration();
    // REQ-SPL-08: absolute per-cycle joint-step ceiling. A smooth Cartesian curve near a
    // singularity can demand unbounded joint speed; refuse to stream such a segment.
    const double max_joint_step_deg = DEFAULT_JOINT_V_MAX.value() * dt.value();

    AxisSet last_ik_seed = start.command.joint_target;
    std::array<Degrees, ROBOT_AXES_COUNT> prev_angles = start.command.joint_target.ToPositionArray();
    const std::array<Degrees, ROBOT_AXES_COUNT> start_angles = prev_angles;
    std::array<Degrees, ROBOT_AXES_COUNT> final_target_angles{};
    std::vector<TrajectoryPoint> points;

    // Max-axis joint step [deg] between the previous rendered joints and a candidate solution.
    const auto maxJointStepDeg = [&prev_angles](const AxisSet& joints) {
        const auto angles = joints.ToPositionArray();
        double step = 0.0;
        for (std::size_t ax = 0; ax < ROBOT_AXES_COUNT; ++ax) {
            step = std::max(step, std::abs(angles[ax].value() - prev_angles[ax].value()));
        }
        return step;
    };

    // Renders one curve sample at time t into a TrajectoryPoint (IK + guard + header stamping).
    // Returns Ok or the typed error to propagate.
    const auto renderPointAt = [&](Seconds t) -> PlannerError {
        auto cart_res = profile->interpolateCartesian(t);
        if (cart_res.isError()) {
            RDT_LOG_ERROR(MODULE_NAME, "Cartesian interpolation failed during SPLINE rendering at t={}", t.toString());
            return PlannerError::InvalidArguments;
        }
        const CartPose cart_pose = cart_res.value();
        auto ik_res = solver_->solveIK(cart_pose, last_ik_seed);
        if (ik_res.isError()) {
            RDT_LOG_ERROR(MODULE_NAME, "IK failed during SPLINE rendering at t={}", t.toString());
            return PlannerError::IK_Failed;
        }
        const double step_deg = maxJointStepDeg(ik_res.value());
        if (step_deg > max_joint_step_deg) {
            RDT_LOG_ERROR(MODULE_NAME,
                "SPLINE block rejected at t={} (step seq {}): a rendered point demands a joint step of "
                "{:.3f} deg per cycle (ceiling {:.3f} deg = DEFAULT_JOINT_V_MAX * dt) — the curve "
                "likely passes near a singularity. The block is NOT planned and the robot holds "
                "position; re-teach the points away from the singular pose and restart.",
                t.toString(),
                block[static_cast<size_t>(profile->blockPointIndexAt(t))].header.sequence_index,
                step_deg, max_joint_step_deg);
            return PlannerError::ExcessiveJointVelocity;
        }

        TrajectoryPoint point;
        // Header of the authored point ending the active span: the HMI executing line follows the
        // block point by point. The stop flag must stay clear on inner points (no stops inside a
        // block, REQ-SPL-02); only the final rendered point is flagged below.
        point.header = block[static_cast<size_t>(profile->blockPointIndexAt(t))].header;
        point.header.is_target_reached_for_this_point = false;
        point.command.joint_target = ik_res.value();
        point.command.cartesian_target = cart_pose;
        point.command.cartesian_valid = true;
        point.command.speed_ratio = block.front().command.speed_ratio;
        prev_angles = ik_res.value().ToPositionArray();
        last_ik_seed = ik_res.value();
        points.push_back(point);
        return PlannerError::Ok;
    };

    if (duration.value() < dt.value()) {
        // Zero-length (robot already on the only authored point) or sub-cycle block: emit the exact
        // endpoint once so execution continuity is preserved (same convention as createSegment).
        if (const PlannerError err = renderPointAt(duration); err != PlannerError::Ok) {
            return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(err);
        }
    } else {
        for (Seconds t = dt; t < duration; t += dt) {
            if (const PlannerError err = renderPointAt(t); err != PlannerError::Ok) {
                return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(err);
            }
        }
        // Exact endpoint, IK re-seeded from the evolved near-endpoint configuration.
        if (const PlannerError err = renderPointAt(duration); err != PlannerError::Ok) {
            return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(err);
        }
    }

    final_target_angles = points.back().command.joint_target.ToPositionArray();
    for (TrajectoryPoint& p : points) {
        p.segment_target.start_angles = start_angles;
        p.segment_target.target_angles = final_target_angles;
        p.segment_target.motion_duration = duration;
        p.segment_target.speed_ratio = block.front().command.speed_ratio;
    }
    points.back().header.is_target_reached_for_this_point = true; // block exit is an exact stop

    RDT_LOG_INFO(MODULE_NAME, "Created SPLINE MotionSegment: {} authored point(s), {} rendered points, duration {}.",
        block.size(), points.size(), duration.toString());

    return Result<std::unique_ptr<MotionSegment>, PlannerError>::Success(
        std::make_unique<MotionSegment>(std::move(points), profile)
    );
}

} // namespace RDT