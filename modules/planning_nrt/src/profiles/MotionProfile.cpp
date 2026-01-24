// MotionProfile.cpp
#include "MotionProfile.h"
#include "LoggingMacros.h"
#include <algorithm>
#include <cmath>

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

// --- MotionProfile Base ---
AxisSet MotionProfile::interpolateJoints(Seconds t) const { (void)t; throw std::logic_error("Joint interpolation not supported by this profile."); }
CartPose MotionProfile::interpolateCartesian(Seconds t) const { (void)t; throw std::logic_error("Cartesian interpolation not supported by this profile."); }

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

AxisSet JointMotionProfile::interpolateJoints(Seconds t) const {
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
    return interp_joints;
}

CartPose JointMotionProfile::interpolateCartesian(Seconds t) const {
    (void)t;
    RDT_LOG_ERROR("JointMotionProfile", "Cartesian interpolation is not valid for a JOINT motion profile.");
    throw std::logic_error("Cannot interpolate Cartesian pose for a JOINT profile.");
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

CartPose LinMotionProfile::interpolateCartesian(Seconds t) const {
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
    return p_interp;
}

AxisSet LinMotionProfile::interpolateJoints(Seconds t) const {
    (void)t;
    RDT_LOG_ERROR("LinMotionProfile", "Joint interpolation is not valid for a LIN motion profile.");
    throw std::logic_error("Cannot interpolate joints for a LIN profile.");
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