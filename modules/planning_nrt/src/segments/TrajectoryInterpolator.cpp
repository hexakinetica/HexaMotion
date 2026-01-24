// TrajectoryInterpolator.cpp
#include "TrajectoryInterpolator.h"
#include "LoggingMacros.h"

namespace RDT {

TrajectoryInterpolator::TrajectoryInterpolator(std::shared_ptr<KinematicSolver> solver)
    : solver_(solver) {
    if (!solver_) {
        RDT_LOG_CRITICAL(MODULE_NAME, "KinematicSolver cannot be null.");
        throw std::invalid_argument("KinematicSolver cannot be null.");
    }
}

Result<std::unique_ptr<MotionSegment>, TrajectoryInterpolator::PlannerError>
TrajectoryInterpolator::createSegment(const TrajectoryPoint& start, const TrajectoryPoint& target, Seconds dt) {
    
    // 1. Create the appropriate motion profile
    std::shared_ptr<MotionProfile> profile;
    try {
        MotionType motion_type = target.header.motion_type;
        double v_factor = std::max(0.01, std::min(1.0, target.command.speed_ratio));
        double a_factor = std::max(0.01, std::min(1.0, target.command.acceleration_ratio));

        if (motion_type == MotionType::JOINT || motion_type == MotionType::PTP) {
            profile = std::make_shared<JointMotionProfile>(
                start.command.joint_target, target.command.joint_target,
                DEFAULT_JOINT_V_MAX * v_factor, DEFAULT_JOINT_A_MAX * a_factor);
        } else if (motion_type == MotionType::LIN) {
            profile = std::make_shared<LinMotionProfile>(
                start.command.cartesian_target, target.command.cartesian_target,
                DEFAULT_CART_V_MAX * v_factor, DEFAULT_CART_A_MAX * a_factor);
        } else {
            RDT_LOG_ERROR(MODULE_NAME, "Unsupported motion type for segment creation: {}", static_cast<int>(motion_type));
            return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::UnsupportedMotionType);
        }
    } catch (const std::exception& e) {
        RDT_LOG_ERROR(MODULE_NAME, "Error creating motion profile: {}", e.what());
        return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::InvalidArguments);
    }

    // 2. "Render" the profile into a vector of points
    std::vector<TrajectoryPoint> points;
    Seconds duration = profile->getDuration();
    AxisSet last_ik_seed = start.command.joint_target;
    
    if (duration.value() < dt.value()) { // Segment is shorter than one time step, or zero length
        if (duration.value() > UnitConstants::DEFAULT_EPSILON) {
             points.push_back(target); // Add the target point if there's any duration
        }
    } else {
        for (Seconds t = dt; t < duration; t += dt) {
            TrajectoryPoint point;
            point.header = target.header;

            if (profile->getMotionType() == MotionType::LIN) {
                CartPose cart_pose = profile->interpolateCartesian(t);
                auto ik_res = solver_->solveIK(cart_pose, last_ik_seed);
                if (ik_res.isError()) {
                    RDT_LOG_ERROR(MODULE_NAME, "IK failed during LIN segment generation at t={}", t.toString());
                    return Result<std::unique_ptr<MotionSegment>, PlannerError>::Failure(PlannerError::IK_Failed);
                }
                point.command.joint_target = ik_res.value();
                last_ik_seed = ik_res.value();
            } else { // JOINT or PTP
                point.command.joint_target = profile->interpolateJoints(t);
            }
            points.push_back(point);
        }
        points.push_back(target); // Always add the final target point to ensure precision
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

} // namespace RDT