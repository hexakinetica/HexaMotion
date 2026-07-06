// MotionProfile.h
#pragma once

#include "DataTypes.h"
#include "Units.h"
#include "SplinePath.h"
#include <Eigen/Geometry>
#include <string>
#include <vector>

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

    /** @brief Gets an interpolated joint-space point at a specific time. Returns a typed error
     *  (instead of throwing, per the project mandate) when the profile does not support joint-space
     *  interpolation (e.g. a Cartesian LIN profile). */
    [[nodiscard]] virtual Result<AxisSet, ErrorCode> interpolateJoints(Seconds t) const = 0;

    /** @brief Gets an interpolated Cartesian-space point at a specific time. Returns a typed error
     *  when the profile does not support Cartesian interpolation (e.g. a joint JOINT/PTP profile). */
    [[nodiscard]] virtual Result<CartPose, ErrorCode> interpolateCartesian(Seconds t) const = 0;
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
    [[nodiscard]] Result<AxisSet, ErrorCode> interpolateJoints(Seconds t) const override;
    [[nodiscard]] Result<CartPose, ErrorCode> interpolateCartesian(Seconds t) const override;

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
    [[nodiscard]] Result<AxisSet, ErrorCode> interpolateJoints(Seconds t) const override;
    [[nodiscard]] Result<CartPose, ErrorCode> interpolateCartesian(Seconds t) const override;

private:
    [[nodiscard]] static Eigen::Quaterniond poseToQuaternion(const CartPose& pose);
    static void quaternionToPoseRot(const Eigen::Quaterniond& q, CartPose& pose);

    CartPose start_pose_, end_pose_;
    Millimeters s_total_displacement_ = 0.0_mm;
    Eigen::Quaterniond q_start_, q_end_slerp_target_;
    TrapezoidalProfileMath profile_math_;

    // CircMotionProfile and SplineMotionProfile reuse the RPY<->quaternion convention of this
    // profile so all Cartesian motion types share one orientation code path (single slerp
    // convention, REQ-CIRC-02 / REQ-SPL-07).
    friend class CircMotionProfile;
    friend class SplineMotionProfile;
};

/**
 * @class CircMotionProfile
 * @brief A motion profile for circular (CIRC) movements in Cartesian space (KUKA-style 3-point arc).
 *
 * The arc starts at `start`, passes through the auxiliary `via` position and ends at `end`
 * (docs/REQ_motion_circ.md). Orientation is slerped start -> end by normalized arc length; the via
 * orientation is ignored (KUKA auxiliary-point semantics). The trapezoidal profile runs on the
 * exact arc length (radius * sweep angle).
 *
 * Constructors must not throw (project mandate), so degenerate input geometry (collinear or
 * coincident points, out-of-range radius) leaves the profile in an explicit INVALID state:
 * isValid() is false, geometryError() names the defect, and every interpolation call returns a
 * typed error. The interpolator checks validity before rendering (fail-closed, REQ-CIRC-04).
 */
class CircMotionProfile : public MotionProfile {
public:
    /** @brief Geometric reasons a 3-point arc cannot be constructed. */
    enum class GeometryError {
        None,             ///< Geometry is valid.
        CoincidentPoints, ///< Two of the three points are closer than kMinPointSeparation.
        CollinearPoints,  ///< The three points do not define a unique circle plane.
        RadiusOutOfRange  ///< The circumradius exceeds kMaxArcRadius (numerically near-collinear).
    };

    /// Minimum pairwise distance between start/via/end for a well-conditioned circle.
    static constexpr Millimeters kMinPointSeparation{0.01};
    /// Circumradius ceiling: beyond this the "circle" is a numerically degenerate near-line.
    static constexpr Millimeters kMaxArcRadius{100000.0}; // 100 m

    CircMotionProfile(const CartPose& start, const CartPose& via, const CartPose& end,
                      MillimetersPerSecond v_max, MillimetersPerSecondSq a_max);

    /** @brief False when the three points do not define a traversable arc; see geometryError(). */
    [[nodiscard]] bool isValid() const { return geometry_error_ == GeometryError::None; }
    [[nodiscard]] GeometryError geometryError() const { return geometry_error_; }
    /** @brief Human-readable name of the geometry defect for diagnostics/logging. */
    [[nodiscard]] static const char* geometryErrorName(GeometryError error);

    [[nodiscard]] Seconds getDuration() const override;
    [[nodiscard]] MotionType getMotionType() const override;
    [[nodiscard]] Result<AxisSet, ErrorCode> interpolateJoints(Seconds t) const override;
    [[nodiscard]] Result<CartPose, ErrorCode> interpolateCartesian(Seconds t) const override;

private:
    CartPose start_pose_, end_pose_;
    GeometryError geometry_error_ = GeometryError::CollinearPoints; // valid only after construction succeeds
    Eigen::Vector3d center_{0.0, 0.0, 0.0}; ///< Circle center [mm].
    Eigen::Vector3d u_hat_{1.0, 0.0, 0.0};  ///< Unit vector center -> start (theta = 0).
    Eigen::Vector3d w_hat_{0.0, 1.0, 0.0};  ///< Unit vector completing the in-plane frame (theta = +90 deg).
    double radius_mm_ = 0.0;                ///< Circumradius [mm].
    double sweep_rad_ = 0.0;                ///< Signed-normalized total arc angle in (0, 2*pi).
    Millimeters arc_length_ = 0.0_mm;       ///< radius * sweep, the trapezoid displacement.
    Eigen::Quaterniond q_start_, q_end_slerp_target_;
    TrapezoidalProfileMath profile_math_;
};

/**
 * @class SplineMotionProfile
 * @brief Motion profile for a SPLINE block: one smooth curve through every block point
 *        (docs/REQ_motion_spline.md).
 *
 * Position follows a centripetal Catmull-Rom SplinePath from the chain start pose through each
 * authored point in order (REQ-SPL-03). Timing comes from a curvature speed plan (REQ-SPL-06
 * rev 2): the path is sampled every kSpeedPlanStepMm of arc length, each sample is speed-limited
 * by the LOCAL curvature (v_lim = min(v_max, sqrt(a_max * R))), and a forward/backward
 * acceleration-limited pass builds the s(t) table. Tight corners are traversed slowly while
 * straight sections keep the commanded speed — the previous single global cap
 * sqrt(a_max * R_min) drove the WHOLE block at the tightest corner's speed and made long blocks
 * crawl end to end. Orientation slerps piecewise between consecutive point orientations by
 * arc-length fraction within each span (REQ-SPL-07; the orientation RATE is only C0 at the
 * points — a documented v1 limitation).
 *
 * Degenerate handling (REQ-SPL-09, constructors must not throw):
 *  - a leading duplicate (start position == first block point, same orientation) is dropped, since
 *    the curve still passes through every authored point;
 *  - a leading duplicate that CHANGES orientation is INVALID (StartCoincidesWithFirstPoint): a
 *    pure reorientation has no arc to travel — that is LIN's job;
 *  - coincident consecutive authored points and an all-duplicate (zero-length) block are INVALID.
 */
class SplineMotionProfile : public MotionProfile {
public:
    /** @brief Reasons a spline block cannot be constructed. */
    enum class GeometryError {
        None,                        ///< Geometry is valid.
        NoPoints,                    ///< The block has no authored points.
        CoincidentPoints,            ///< Consecutive authored points closer than the minimum separation.
        StartCoincidesWithFirstPoint ///< Start == first point but orientation differs (pure reorientation).
    };

    /// Orientation delta below which a leading duplicate point is considered a true duplicate [deg].
    static constexpr Degrees kMaxLeadingDuplicateOrientationDelta{0.01};

    /// Arc-length step of the curvature speed plan [mm] (REQ-SPL-06 rev 2). Fine enough to
    /// resolve teach-scale corners; a 1 m block costs ~500 plan samples at plan time only.
    static constexpr double kSpeedPlanStepMm = 2.0;
    /// Lower bound of the planned path speed [mm/s]: bounds the block duration on a near-cusp
    /// sample instead of letting one degenerate radius stretch the plan unbounded.
    static constexpr double kMinPlannedSpeedMmS = 1.0;

    SplineMotionProfile(const CartPose& start, const std::vector<CartPose>& block_points,
                        MillimetersPerSecond v_max, MillimetersPerSecondSq a_max);

    [[nodiscard]] bool isValid() const { return geometry_error_ == GeometryError::None; }
    [[nodiscard]] GeometryError geometryError() const { return geometry_error_; }
    [[nodiscard]] static const char* geometryErrorName(GeometryError error);

    /**
     * @brief Index of the authored block point that ends the span active at time @p t.
     * Drives per-point program-line tracking during rendering (the HMI line advances as the block
     * passes each authored point). Valid only on a valid profile.
     */
    [[nodiscard]] int blockPointIndexAt(Seconds t) const;

    [[nodiscard]] Seconds getDuration() const override;
    [[nodiscard]] MotionType getMotionType() const override;
    [[nodiscard]] Result<AxisSet, ErrorCode> interpolateJoints(Seconds t) const override;
    [[nodiscard]] Result<CartPose, ErrorCode> interpolateCartesian(Seconds t) const override;

private:
    /// Builds the curvature speed plan (REQ-SPL-06 rev 2): samples the path every
    /// kSpeedPlanStepMm, limits each sample by local curvature, runs the forward/backward
    /// acceleration passes and fills speed_plan_s_mm_/speed_plan_t_s_/total_duration_.
    void buildSpeedPlan(double v_max_mm_s, double a_max_mm_s2);

    /// Path length [mm] reached at time @p t through the speed plan (clamped to the block).
    [[nodiscard]] double pathLengthAt(Seconds t) const;

    GeometryError geometry_error_ = GeometryError::NoPoints;
    SplinePath path_{{}};                        ///< Invalid until construction succeeds.
    // Zero-length block (robot already at the only authored point): valid no-op profile with
    // duration 0, mirroring LinMotionProfile's zero-displacement behavior — not an error.
    bool zero_length_ = false;
    CartPose zero_length_pose_{};
    std::vector<Eigen::Quaterniond> curve_orientations_; ///< One quaternion per curve point (spans + 1),
                                                         ///< hemisphere-aligned span by span.
    std::vector<int> span_block_index_;                  ///< Authored block index ending each span.
    // Curvature speed plan (REQ-SPL-06 rev 2): matching path-length [mm] / time [s] samples of
    // s(t), built once at construction; interpolation resolves both directions monotonically.
    std::vector<double> speed_plan_s_mm_;
    std::vector<double> speed_plan_t_s_;
    Seconds total_duration_{0.0};
};

} // namespace RDT