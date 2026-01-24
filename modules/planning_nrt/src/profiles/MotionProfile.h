
// MotionProfile.h
#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#pragma once

#include "DataTypes.h"
#include "Units.h"
#include <Eigen/Geometry>
#include <string>

namespace RDT {

/**
 * @struct TrapParams
 * @brief Stores the calculated time parameters for a trapezoidal velocity profile.
 */
struct TrapParams {
    Seconds t_accel = 0.0_s;        ///< Duration of the acceleration phase.
    Seconds t_const_vel = 0.0_s;    ///< Duration of the constant velocity phase.
    Seconds t_decel = 0.0_s;        ///< Duration of the deceleration phase.
    double peak_vel_value = 0.0;    ///< The maximum velocity reached.
    Seconds total_duration = 0.0_s;   ///< Total duration of the profile (t_accel + t_const_vel + t_decel).
    double actual_accel_value = 0.0;///< The acceleration value used for calculations.
};

/**
 * @class TrapezoidalProfileMath
 * @brief A stateless utility class for calculating trapezoidal motion profile parameters and positions.
 */
class TrapezoidalProfileMath {
public:
    TrapezoidalProfileMath() = default;

    /**
     * @brief Calculates the parameters of the trapezoidal profile.
     * @param displacement_val The total distance or angle to travel (must be non-negative).
     * @param v_limit_val The maximum velocity limit (must be positive for movement).
     * @param a_limit_val The maximum acceleration limit (must be positive for acceleration).
     */
    void calculate(double displacement_val, double v_limit_val, double a_limit_val);

    /**
     * @brief Gets the position along the path at a given time.
     * @param t Time elapsed since the start of the profile.
     * @return The position (distance or angle) at time t.
     */
    [[nodiscard]] double getPositionAt(Seconds t) const;
    
    /**
     * @brief Gets the calculated profile parameters.
     */
    [[nodiscard]] const TrapParams& getParams() const;

private:
    TrapParams params_;
    double displacement_val_ = 0.0;
};

/**
 * @class MotionProfile
 * @brief Abstract base class for different types of motion profiles (e.g., LIN, PTP).
 */
class MotionProfile {
public:
    virtual ~MotionProfile() = default;

    /** @brief Returns the total duration of the motion. */
    [[nodiscard]] virtual Seconds getDuration() const = 0;

    /** @brief Returns the type of motion. */
    [[nodiscard]] virtual MotionType getMotionType() const = 0;

    /** @brief Gets an interpolated joint-space point at a specific time. */
    [[nodiscard]] virtual AxisSet interpolateJoints(Seconds t) const = 0;

    /** @brief Gets an interpolated Cartesian-space point at a specific time. */
    [[nodiscard]] virtual CartPose interpolateCartesian(Seconds t) const = 0;
};

/**
 * @class JointMotionProfile
 * @brief A motion profile for point-to-point (PTP/JOINT) movements in joint space.
 */
class JointMotionProfile : public MotionProfile {
public:
    JointMotionProfile(const AxisSet& start, const AxisSet& end, DegreesPerSecond v_max, DegreesPerSecondSq a_max);

    [[nodiscard]] Seconds getDuration() const override;
    [[nodiscard]] MotionType getMotionType() const override;
    [[nodiscard]] AxisSet interpolateJoints(Seconds t) const override;
    [[nodiscard]] CartPose interpolateCartesian(Seconds t) const override;

private:
    AxisSet start_joints_, end_joints_;
    Degrees max_angular_delta_ = 0.0_deg;
    TrapezoidalProfileMath profile_math_;
};

/**
 * @class LinMotionProfile
 * @brief A motion profile for linear (LIN) movements in Cartesian space.
 */
class LinMotionProfile : public MotionProfile {
public:
    LinMotionProfile(const CartPose& start, const CartPose& end, MillimetersPerSecond v_max, MillimetersPerSecondSq a_max);

    [[nodiscard]] Seconds getDuration() const override;
    [[nodiscard]] MotionType getMotionType() const override;
    [[nodiscard]] AxisSet interpolateJoints(Seconds t) const override;
    [[nodiscard]] CartPose interpolateCartesian(Seconds t) const override;

private:
    [[nodiscard]] static Eigen::Quaterniond poseToQuaternion(const CartPose& pose);
    static void quaternionToPoseRot(const Eigen::Quaterniond& q, CartPose& pose);

    CartPose start_pose_, end_pose_;
    Millimeters s_total_displacement_ = 0.0_mm;
    Eigen::Quaterniond q_start_, q_end_slerp_target_;
    TrapezoidalProfileMath profile_math_;
};

} // namespace RDT

#endif // MOTION_PROFILE_H