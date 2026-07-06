// MotionProfile.cpp
#include "MotionProfile.h"
#include "LoggingMacros.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace RDT {

// --- TrapezoidalProfileMath ---
void TrapezoidalProfileMath::calculate(double displacement, double v_limit, double a_limit) {
    params_ = {}; 
    displacement_val_ = std::abs(displacement);
    double v_limit_val = std::abs(v_limit);
    double a_limit_val = std::abs(a_limit);

    if (displacement_val_ < UnitConstants::DEFAULT_EPSILON) {
        return; 
    }
    if (v_limit_val < UnitConstants::DEFAULT_EPSILON || a_limit_val < UnitConstants::DEFAULT_EPSILON) {
        // Cannot accelerate, assume constant velocity if possible
        if (v_limit_val > UnitConstants::DEFAULT_EPSILON) {
            params_.total_duration = Seconds(displacement_val_ / v_limit_val);
        }
        return;
    }

    // Time to reach max velocity
    Seconds t_acc_to_v_limit = Seconds(v_limit_val / a_limit_val);
    // Distance covered during full acceleration and deceleration
    double dist_acc_dec_full = v_limit_val * v_limit_val / a_limit_val;

    if (dist_acc_dec_full <= displacement_val_) { // Trapezoidal profile
        params_.peak_vel_value = v_limit_val;
        params_.t_accel = t_acc_to_v_limit;
        params_.t_decel = t_acc_to_v_limit;
        params_.t_const_vel = Seconds((displacement_val_ - dist_acc_dec_full) / v_limit_val);
        params_.actual_accel_value = a_limit_val;
    } else { // Triangular profile (max velocity is not reached)
        params_.t_accel = Seconds(std::sqrt(displacement_val_ / a_limit_val));
        params_.t_decel = params_.t_accel;
        params_.peak_vel_value = a_limit_val * params_.t_accel.value();
        params_.t_const_vel = 0.0_s;
        params_.actual_accel_value = a_limit_val;
    }

    params_.total_duration = params_.t_accel + params_.t_const_vel + params_.t_decel;
}

double TrapezoidalProfileMath::getPositionAt(Seconds t) const {
    if (displacement_val_ < UnitConstants::DEFAULT_EPSILON || params_.total_duration < (UnitConstants::DEFAULT_EPSILON * 1.0_s)) {
        return (t >= params_.total_duration) ? displacement_val_ : 0.0;
    }
    Seconds clamped_t = std::max(0.0_s, std::min(t, params_.total_duration));
    double time_s = clamped_t.value();

    if (clamped_t <= params_.t_accel) {
        return 0.5 * params_.actual_accel_value * time_s * time_s;
    } else if (clamped_t <= params_.t_accel + params_.t_const_vel) {
        double s_at_accel_end = 0.5 * params_.actual_accel_value * params_.t_accel.value() * params_.t_accel.value();
        return s_at_accel_end + params_.peak_vel_value * (time_s - params_.t_accel.value());
    } else {
        double s_at_accel_end = 0.5 * params_.actual_accel_value * params_.t_accel.value() * params_.t_accel.value();
        double s_at_const_end = s_at_accel_end + params_.peak_vel_value * params_.t_const_vel.value();
        double time_in_decel = time_s - (params_.t_accel.value() + params_.t_const_vel.value());
        return s_at_const_end + (params_.peak_vel_value * time_in_decel - 0.5 * params_.actual_accel_value * time_in_decel * time_in_decel);
    }
}
const TrapParams& TrapezoidalProfileMath::getParams() const { return params_; }

// Base interpolateJoints/interpolateCartesian are pure virtual (=0) with no shared behaviour, so the
// previous throwing definitions were dead code and were removed. Each concrete profile implements
// the interpolation it supports and returns a typed error from the one it does not.

// --- JointMotionProfile ---
JointMotionProfile::JointMotionProfile(const AxisSet& start, const AxisSet& end, DegreesPerSecond v_max, DegreesPerSecondSq a_max)
    : start_joints_(start), end_joints_(end) {
    Degrees max_delta_val = 0.0_deg;
    for (std::size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        max_delta_val = std::max(max_delta_val, (end_joints_[static_cast<AxisId>(i)].position - start_joints_[static_cast<AxisId>(i)].position).abs());
    }
    max_angular_delta_ = max_delta_val;
    profile_math_.calculate(max_angular_delta_.value(), v_max.value(), a_max.value());
}

Seconds JointMotionProfile::getDuration() const { return profile_math_.getParams().total_duration; }
MotionType JointMotionProfile::getMotionType() const { return MotionType::JOINT; }

Result<AxisSet, ErrorCode> JointMotionProfile::interpolateJoints(Seconds t) const {
    double s_path_norm_factor = 0.0;
    if (max_angular_delta_ > (UnitConstants::DEFAULT_EPSILON * 1.0_deg)) {
        s_path_norm_factor = profile_math_.getPositionAt(t) / max_angular_delta_.value();
    } else {
        s_path_norm_factor = (t >= getDuration()) ? 1.0 : 0.0;
    }
    s_path_norm_factor = std::max(0.0, std::min(s_path_norm_factor, 1.0));

    AxisSet interp_joints;
    for (std::size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        AxisId axis = static_cast<AxisId>(i);
        interp_joints[axis].position = start_joints_[axis].position + (end_joints_[axis].position - start_joints_[axis].position) * s_path_norm_factor;
    }
    return Result<AxisSet, ErrorCode>::Success(interp_joints);
}

Result<CartPose, ErrorCode> JointMotionProfile::interpolateCartesian(Seconds t) const {
    (void)t;
    // A JOINT/PTP profile has no Cartesian path. Callers dispatch on getMotionType(), so this is a
    // guard against a wrong-type call; report it as a typed error instead of throwing.
    RDT_LOG_ERROR("JointMotionProfile", "Cartesian interpolation is not valid for a JOINT motion profile.");
    return Result<CartPose, ErrorCode>::Failure(ErrorCode::InvalidArgument);
}

// --- LinMotionProfile ---
LinMotionProfile::LinMotionProfile(const CartPose& start, const CartPose& end, MillimetersPerSecond v_max, MillimetersPerSecondSq a_max)
    : start_pose_(start), end_pose_(end) {
    Millimeters dx = end_pose_.x - start_pose_.x;
    Millimeters dy = end_pose_.y - start_pose_.y;
    Millimeters dz = end_pose_.z - start_pose_.z;
    s_total_displacement_ = Millimeters(std::sqrt(dx.value()*dx.value() + dy.value()*dy.value() + dz.value()*dz.value()));
    
    q_start_ = poseToQuaternion(start_pose_);
    Eigen::Quaterniond q_end_temp = poseToQuaternion(end_pose_);
    if (q_start_.dot(q_end_temp) < 0.0) {
        q_end_slerp_target_.coeffs() = -q_end_temp.coeffs();
    } else {
        q_end_slerp_target_ = q_end_temp;
    }

    profile_math_.calculate(s_total_displacement_.value(), v_max.value(), a_max.value());
}

Seconds LinMotionProfile::getDuration() const { return profile_math_.getParams().total_duration; }
MotionType LinMotionProfile::getMotionType() const { return MotionType::LIN; }

Result<CartPose, ErrorCode> LinMotionProfile::interpolateCartesian(Seconds t) const {
    double alpha = 0.0;
    if (s_total_displacement_ > (UnitConstants::DEFAULT_EPSILON * 1.0_mm)) {
        double s_path_on_profile = profile_math_.getPositionAt(t);
        alpha = s_path_on_profile / s_total_displacement_.value();
    } else if (getDuration() > (UnitConstants::DEFAULT_EPSILON * 1.0_s)) {
        // Pure orientation change
        alpha = t.value() / getDuration().value();
    } else {
        alpha = (t >= getDuration()) ? 1.0 : 0.0;
    }
    alpha = std::max(0.0, std::min(alpha, 1.0));
    
    CartPose p_interp;
    p_interp.x = start_pose_.x + (end_pose_.x - start_pose_.x) * alpha;
    p_interp.y = start_pose_.y + (end_pose_.y - start_pose_.y) * alpha;
    p_interp.z = start_pose_.z + (end_pose_.z - start_pose_.z) * alpha;

    Eigen::Quaterniond q_interpolated = q_start_.slerp(alpha, q_end_slerp_target_);
    quaternionToPoseRot(q_interpolated.normalized(), p_interp);
    return Result<CartPose, ErrorCode>::Success(p_interp);
}

Result<AxisSet, ErrorCode> LinMotionProfile::interpolateJoints(Seconds t) const {
    (void)t;
    // A LIN profile is Cartesian; joint interpolation is done by the interpolator via IK. Callers
    // dispatch on getMotionType(), so this guards a wrong-type call with a typed error, not a throw.
    RDT_LOG_ERROR("LinMotionProfile", "Joint interpolation is not valid for a LIN motion profile.");
    return Result<AxisSet, ErrorCode>::Failure(ErrorCode::InvalidArgument);
}

// --- CircMotionProfile ---
namespace {
// Positions of a CartPose as an Eigen vector in millimeters (positions only; orientation is
// handled separately by the quaternion slerp).
Eigen::Vector3d posePosition(const CartPose& pose) {
    return Eigen::Vector3d(pose.x.value(), pose.y.value(), pose.z.value());
}
} // namespace

CircMotionProfile::CircMotionProfile(const CartPose& start, const CartPose& via, const CartPose& end,
                                     MillimetersPerSecond v_max, MillimetersPerSecondSq a_max)
    : start_pose_(start), end_pose_(end) {
    const Eigen::Vector3d p_start = posePosition(start);
    const Eigen::Vector3d p_via = posePosition(via);
    const Eigen::Vector3d p_end = posePosition(end);

    // 1. Coincidence guard: a circle through three points is well-conditioned only when the points
    //    are pairwise separated. This also rejects the zero-length "pure orientation change" case,
    //    which is a LIN capability, not a CIRC one (REQ-CIRC-04/06).
    const double min_sep = kMinPointSeparation.value();
    if ((p_via - p_start).norm() < min_sep ||
        (p_end - p_via).norm() < min_sep ||
        (p_end - p_start).norm() < min_sep) {
        geometry_error_ = GeometryError::CoincidentPoints;
        return;
    }

    // 2. Circumcenter of the triangle (start, via, end) in 3D:
    //    center = start + ((|a|^2 * b - |b|^2 * a) x (a x b)) / (2 * |a x b|^2),
    //    where a = via - start, b = end - start. |a x b| ~ 0 means collinear points: no unique
    //    circle plane exists.
    const Eigen::Vector3d a = p_via - p_start;
    const Eigen::Vector3d b = p_end - p_start;
    const Eigen::Vector3d cross_ab = a.cross(b);
    const double cross_norm_sq = cross_ab.squaredNorm();
    // Relative collinearity test: sin(angle between a and b) below ~1e-6 has no stable plane.
    constexpr double kMinPlaneSinAngle = 1.0e-6;
    if (cross_ab.norm() < kMinPlaneSinAngle * a.norm() * b.norm()) {
        geometry_error_ = GeometryError::CollinearPoints;
        return;
    }
    center_ = p_start + (a.squaredNorm() * b - b.squaredNorm() * a).cross(cross_ab) / (2.0 * cross_norm_sq);
    radius_mm_ = (p_start - center_).norm();

    // 3. Radius sanity: a near-collinear triple yields a gigantic circumradius whose arc is
    //    numerically a straight line. Reject it explicitly instead of executing garbage.
    if (radius_mm_ > kMaxArcRadius.value() || radius_mm_ < min_sep) {
        geometry_error_ = GeometryError::RadiusOutOfRange;
        return;
    }

    // 4. In-plane traversal frame with theta(start) = 0. The plane normal candidate is a x b; the
    //    traversal direction must visit via strictly before end, otherwise the frame is flipped so
    //    the arc runs the other way around the circle (the via point selects the direction,
    //    REQ-CIRC-01).
    u_hat_ = (p_start - center_).normalized();
    Eigen::Vector3d n_hat = cross_ab.normalized();
    w_hat_ = n_hat.cross(u_hat_);

    // Angle of a point on the circle in the (u_hat, w_hat) frame, normalized to [0, 2*pi).
    constexpr double kTwoPi = 2.0 * std::numbers::pi;
    const auto angle_of = [this, kTwoPi](const Eigen::Vector3d& p) {
        const Eigen::Vector3d r = p - center_;
        double theta = std::atan2(r.dot(w_hat_), r.dot(u_hat_));
        if (theta < 0.0) {
            theta += kTwoPi;
        }
        return theta;
    };

    const double theta_via = angle_of(p_via);
    double theta_end = angle_of(p_end);
    if (theta_via > theta_end) {
        // Via lies beyond the end going this way around: traverse the opposite direction. Flipping
        // w_hat maps every angle theta to 2*pi - theta, which swaps the order of via and end (via
        // then lands strictly inside the new sweep, so it stays on the traversed arc).
        w_hat_ = -w_hat_;
        theta_end = kTwoPi - theta_end;
    }
    sweep_rad_ = theta_end;
    arc_length_ = Millimeters(radius_mm_ * sweep_rad_);

    // 5. Orientation: slerp start -> end with the shortest-path hemisphere fix, identical to LIN
    //    (REQ-CIRC-02). The via orientation is intentionally ignored (KUKA auxiliary point).
    q_start_ = LinMotionProfile::poseToQuaternion(start_pose_);
    Eigen::Quaterniond q_end_temp = LinMotionProfile::poseToQuaternion(end_pose_);
    if (q_start_.dot(q_end_temp) < 0.0) {
        q_end_slerp_target_.coeffs() = -q_end_temp.coeffs();
    } else {
        q_end_slerp_target_ = q_end_temp;
    }

    profile_math_.calculate(arc_length_.value(), v_max.value(), a_max.value());
    geometry_error_ = GeometryError::None;
}

const char* CircMotionProfile::geometryErrorName(GeometryError error) {
    switch (error) {
        case GeometryError::None: return "None";
        case GeometryError::CoincidentPoints: return "CoincidentPoints (start/via/end closer than the minimum separation)";
        case GeometryError::CollinearPoints: return "CollinearPoints (no unique circle plane through start/via/end)";
        case GeometryError::RadiusOutOfRange: return "RadiusOutOfRange (near-collinear points, circumradius outside sane bounds)";
    }
    return "UnknownGeometryError";
}

Seconds CircMotionProfile::getDuration() const { return profile_math_.getParams().total_duration; }
MotionType CircMotionProfile::getMotionType() const { return MotionType::CIRC; }

Result<CartPose, ErrorCode> CircMotionProfile::interpolateCartesian(Seconds t) const {
    if (!isValid()) {
        // Guard against use-after-failed-construction: the interpolator must have rejected this
        // profile already, so reaching here is a programming error — report, do not crash.
        RDT_LOG_ERROR("CircMotionProfile", "interpolateCartesian called on an INVALID arc: {}.",
                      geometryErrorName(geometry_error_));
        return Result<CartPose, ErrorCode>::Failure(ErrorCode::InvalidArgument);
    }

    double alpha = 0.0;
    if (arc_length_ > (UnitConstants::DEFAULT_EPSILON * 1.0_mm)) {
        alpha = profile_math_.getPositionAt(t) / arc_length_.value();
    } else {
        alpha = (t >= getDuration()) ? 1.0 : 0.0;
    }
    alpha = std::max(0.0, std::min(alpha, 1.0));

    const double theta = alpha * sweep_rad_;
    const Eigen::Vector3d position =
        center_ + radius_mm_ * (std::cos(theta) * u_hat_ + std::sin(theta) * w_hat_);

    CartPose p_interp;
    p_interp.x = Millimeters(position.x());
    p_interp.y = Millimeters(position.y());
    p_interp.z = Millimeters(position.z());

    Eigen::Quaterniond q_interpolated = q_start_.slerp(alpha, q_end_slerp_target_);
    LinMotionProfile::quaternionToPoseRot(q_interpolated.normalized(), p_interp);
    return Result<CartPose, ErrorCode>::Success(p_interp);
}

Result<AxisSet, ErrorCode> CircMotionProfile::interpolateJoints(Seconds t) const {
    (void)t;
    // A CIRC profile is Cartesian; joint interpolation is done by the interpolator via IK. Callers
    // dispatch on getMotionType(), so this guards a wrong-type call with a typed error, not a throw.
    RDT_LOG_ERROR("CircMotionProfile", "Joint interpolation is not valid for a CIRC motion profile.");
    return Result<AxisSet, ErrorCode>::Failure(ErrorCode::InvalidArgument);
}

// --- SplineMotionProfile ---
SplineMotionProfile::SplineMotionProfile(const CartPose& start, const std::vector<CartPose>& block_points,
                                         MillimetersPerSecond v_max, MillimetersPerSecondSq a_max) {
    if (block_points.empty()) {
        geometry_error_ = GeometryError::NoPoints;
        return;
    }

    // Curve point list: the chain start pose followed by every authored point. A leading duplicate
    // (start already at the first authored point) is dropped so the knot intervals stay
    // well-conditioned — the curve still passes through every AUTHORED point. If that duplicate
    // changes orientation, the block would be a pure reorientation with no arc to travel: that is
    // LIN's job, so it is rejected as a typed error instead of guessing a rotation timing.
    std::vector<CartPose> curve_poses;
    curve_poses.reserve(block_points.size() + 1);
    span_block_index_.clear();

    const Eigen::Vector3d p_start(start.x.value(), start.y.value(), start.z.value());
    const Eigen::Vector3d p_first(block_points.front().x.value(), block_points.front().y.value(),
                                  block_points.front().z.value());
    const bool leading_duplicate = (p_first - p_start).norm() < SplinePath::kMinPointSeparation.value();
    if (leading_duplicate) {
        const Eigen::Quaterniond q_s = LinMotionProfile::poseToQuaternion(start);
        const Eigen::Quaterniond q_f = LinMotionProfile::poseToQuaternion(block_points.front());
        const double delta_deg = Degrees::fromRadians(q_s.angularDistance(q_f)).value();
        if (delta_deg > kMaxLeadingDuplicateOrientationDelta.value()) {
            geometry_error_ = GeometryError::StartCoincidesWithFirstPoint;
            return;
        }
    } else {
        curve_poses.push_back(start);
    }
    curve_poses.insert(curve_poses.end(), block_points.begin(), block_points.end());

    if (curve_poses.size() < 2) {
        // Single-point block with the robot already on it: a legal no-op, not a defect
        // (mirrors LIN's zero-displacement handling). Trivial one-sample plan, duration 0.
        zero_length_ = true;
        zero_length_pose_ = curve_poses.front();
        speed_plan_s_mm_ = {0.0};
        speed_plan_t_s_ = {0.0};
        total_duration_ = Seconds(0.0);
        geometry_error_ = GeometryError::None;
        return;
    }

    // Authored index that ends each span: span s ends at curve point s+1. When the start pose is
    // curve point 0, curve point k >= 1 is authored point k-1; after a leading-duplicate drop,
    // curve point k is authored point k.
    const int authored_offset = leading_duplicate ? 0 : -1;
    for (size_t k = 1; k < curve_poses.size(); ++k) {
        span_block_index_.push_back(static_cast<int>(k) + authored_offset);
    }

    std::vector<Eigen::Vector3d> positions;
    positions.reserve(curve_poses.size());
    for (const CartPose& pose : curve_poses) {
        positions.emplace_back(pose.x.value(), pose.y.value(), pose.z.value());
    }
    path_ = SplinePath(positions);
    if (!path_.isValid()) {
        // TooFewPoints cannot occur here (size checked above), so any failure is a coincidence
        // between authored points — the operator's program defect (REQ-SPL-09).
        geometry_error_ = GeometryError::CoincidentPoints;
        return;
    }

    // One quaternion per curve point, hemisphere-aligned to its predecessor so every per-span
    // slerp takes the short way (same convention as LIN/CIRC).
    curve_orientations_.reserve(curve_poses.size());
    for (const CartPose& pose : curve_poses) {
        Eigen::Quaterniond q = LinMotionProfile::poseToQuaternion(pose);
        if (!curve_orientations_.empty() && curve_orientations_.back().dot(q) < 0.0) {
            q.coeffs() = -q.coeffs();
        }
        curve_orientations_.push_back(q);
    }

    // Curvature speed plan (REQ-SPL-06 rev 2): pointwise limiting instead of the previous global
    // sqrt(a_max * R_min) cap, which drove the WHOLE block at the tightest corner's speed and
    // made long blocks crawl end to end.
    buildSpeedPlan(v_max.value(), a_max.value());
    geometry_error_ = GeometryError::None;
}

void SplineMotionProfile::buildSpeedPlan(double v_max_mm_s, double a_max_mm_s2) {
    const double length = path_.totalLength();
    // At least two samples (block entry and exit); uniform arc-length grid, last sample exactly L.
    const std::size_t interval_count =
        std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(length / kSpeedPlanStepMm)));
    const double ds = length / static_cast<double>(interval_count);
    const std::size_t sample_count = interval_count + 1;

    speed_plan_s_mm_.resize(sample_count);
    for (std::size_t i = 0; i < sample_count; ++i) {
        speed_plan_s_mm_[i] = ds * static_cast<double>(i);
    }
    speed_plan_s_mm_.back() = length;

    // 1. Local curvature speed limit per sample. R is the circumradius of the curve points one
    //    grid step around the sample (R = |AB||BC||CA| / (2 |cross(AB, AC)|)); collinear samples
    //    report the straight-path radius so the limit stays at v_max.
    std::vector<double> v_limit(sample_count, v_max_mm_s);
    for (std::size_t i = 1; i + 1 < sample_count; ++i) {
        const Eigen::Vector3d a = path_.positionAt(speed_plan_s_mm_[i - 1]);
        const Eigen::Vector3d b = path_.positionAt(speed_plan_s_mm_[i]);
        const Eigen::Vector3d c = path_.positionAt(speed_plan_s_mm_[i + 1]);
        const double cross_norm = (b - a).cross(c - a).norm();
        double radius_mm = SplinePath::kStraightCurvatureRadius;
        if (cross_norm > 1.0e-12) {
            radius_mm = ((b - a).norm() * (c - b).norm() * (c - a).norm()) / (2.0 * cross_norm);
        }
        const double v_curvature = std::sqrt(a_max_mm_s2 * radius_mm);
        v_limit[i] = std::max(kMinPlannedSpeedMmS, std::min(v_max_mm_s, v_curvature));
    }
    // Rest-to-rest block entry/exit (REQ-SPL-02: exact stops at the block boundaries).
    v_limit.front() = 0.0;
    v_limit.back() = 0.0;

    // 2. Forward/backward acceleration-limited passes (a_max tangential): the standard two-pass
    //    profile limiter. After both passes v[] is reachable from rest in both directions and
    //    never exceeds the local curvature limit.
    std::vector<double> v = v_limit;
    for (std::size_t i = 1; i < sample_count; ++i) {
        const double v_reachable = std::sqrt(v[i - 1] * v[i - 1] + 2.0 * a_max_mm_s2 * ds);
        v[i] = std::min(v[i], v_reachable);
    }
    for (std::size_t i = sample_count - 1; i > 0; --i) {
        const double v_brakeable = std::sqrt(v[i] * v[i] + 2.0 * a_max_mm_s2 * ds);
        v[i - 1] = std::min(v[i - 1], v_brakeable);
    }

    // 3. Integrate the time table over the grid (trapezoidal average per interval). The average
    //    is floored so the two rest endpoints cannot produce a division blow-up; every interior
    //    sample is already >= kMinPlannedSpeedMmS.
    speed_plan_t_s_.resize(sample_count);
    speed_plan_t_s_[0] = 0.0;
    for (std::size_t i = 1; i < sample_count; ++i) {
        const double v_avg = std::max(0.5 * (v[i - 1] + v[i]), 0.5 * kMinPlannedSpeedMmS);
        speed_plan_t_s_[i] = speed_plan_t_s_[i - 1] + ds / v_avg;
    }
    total_duration_ = Seconds(speed_plan_t_s_.back());
}

double SplineMotionProfile::pathLengthAt(Seconds t) const {
    if (speed_plan_t_s_.size() < 2 || t.value() <= 0.0) {
        return 0.0;
    }
    const double t_s = t.value();
    if (t_s >= speed_plan_t_s_.back()) {
        return speed_plan_s_mm_.back();
    }
    // The time table is strictly increasing (every interval gets a positive dt), so the upper
    // bound is always an interior sample here and the interval below it is well-formed.
    const auto it = std::upper_bound(speed_plan_t_s_.begin(), speed_plan_t_s_.end(), t_s);
    const std::size_t hi = static_cast<std::size_t>(it - speed_plan_t_s_.begin());
    const std::size_t lo = hi - 1;
    const double span_dt = speed_plan_t_s_[hi] - speed_plan_t_s_[lo];
    const double fraction = span_dt > 0.0 ? (t_s - speed_plan_t_s_[lo]) / span_dt : 1.0;
    return speed_plan_s_mm_[lo] + fraction * (speed_plan_s_mm_[hi] - speed_plan_s_mm_[lo]);
}

const char* SplineMotionProfile::geometryErrorName(GeometryError error) {
    switch (error) {
        case GeometryError::None: return "None";
        case GeometryError::NoPoints: return "NoPoints (the spline block has no authored points)";
        case GeometryError::CoincidentPoints:
            return "CoincidentPoints (consecutive spline points closer than the minimum separation)";
        case GeometryError::StartCoincidesWithFirstPoint:
            return "StartCoincidesWithFirstPoint (first spline point equals the start position but "
                   "changes orientation; use LIN for a pure reorientation)";
    }
    return "UnknownGeometryError";
}

int SplineMotionProfile::blockPointIndexAt(Seconds t) const {
    if (!isValid() || zero_length_ || span_block_index_.empty()) {
        return 0;
    }
    const SplinePath::SpanLocation loc = path_.spanLocationAt(pathLengthAt(t));
    const size_t span = static_cast<size_t>(std::clamp(loc.span, 0, static_cast<int>(span_block_index_.size()) - 1));
    return span_block_index_[span];
}

Seconds SplineMotionProfile::getDuration() const { return total_duration_; }
MotionType SplineMotionProfile::getMotionType() const { return MotionType::SPLINE; }

Result<CartPose, ErrorCode> SplineMotionProfile::interpolateCartesian(Seconds t) const {
    if (!isValid()) {
        // Guard against use-after-failed-construction (the interpolator must have rejected this
        // profile already): report, do not crash.
        RDT_LOG_ERROR("SplineMotionProfile", "interpolateCartesian called on an INVALID spline: {}.",
                      geometryErrorName(geometry_error_));
        return Result<CartPose, ErrorCode>::Failure(ErrorCode::InvalidArgument);
    }
    if (zero_length_) {
        return Result<CartPose, ErrorCode>::Success(zero_length_pose_);
    }

    const double s = pathLengthAt(t);
    const Eigen::Vector3d position = path_.positionAt(s);
    const SplinePath::SpanLocation loc = path_.spanLocationAt(s);

    CartPose p_interp;
    p_interp.x = Millimeters(position.x());
    p_interp.y = Millimeters(position.y());
    p_interp.z = Millimeters(position.z());

    const size_t q_lo = static_cast<size_t>(loc.span);
    const Eigen::Quaterniond q_interpolated =
        curve_orientations_[q_lo].slerp(loc.fraction, curve_orientations_[q_lo + 1]);
    LinMotionProfile::quaternionToPoseRot(q_interpolated.normalized(), p_interp);
    return Result<CartPose, ErrorCode>::Success(p_interp);
}

Result<AxisSet, ErrorCode> SplineMotionProfile::interpolateJoints(Seconds t) const {
    (void)t;
    // A SPLINE profile is Cartesian; joint interpolation is done by the interpolator via IK.
    RDT_LOG_ERROR("SplineMotionProfile", "Joint interpolation is not valid for a SPLINE motion profile.");
    return Result<AxisSet, ErrorCode>::Failure(ErrorCode::InvalidArgument);
}

Eigen::Quaterniond LinMotionProfile::poseToQuaternion(const CartPose& pose) {
    return Eigen::AngleAxisd(pose.rz.toRadians(), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(pose.ry.toRadians(), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(pose.rx.toRadians(), Eigen::Vector3d::UnitX());
}

void LinMotionProfile::quaternionToPoseRot(const Eigen::Quaterniond& q, CartPose& pose) {
    Eigen::Vector3d euler_angles_rad = q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX order for RPY
    pose.rz = Degrees::fromRadians(euler_angles_rad[0]);
    pose.ry = Degrees::fromRadians(euler_angles_rad[1]);
    pose.rx = Degrees::fromRadians(euler_angles_rad[2]);
}

} // namespace RDT