// SplinePath.h
#pragma once

#include "Units.h"
#include <Eigen/Geometry> // Vector3d + cross() (curvature circumradius)
#include <vector>

namespace RDT {

/**
 * @class SplinePath
 * @brief Interpolating centripetal Catmull-Rom position path through N >= 2 points
 *        (docs/REQ_motion_spline.md, REQ-SPL-03/04).
 *
 * The curve passes exactly through every input point. The centripetal knot parametrization
 * (knot increment = sqrt of chord length) produces no cusps or self-intersections within a span,
 * and the spline is local: moving one point reshapes only the spans that reference it. Endpoint
 * tangents come from phantom-point reflection (P_-1 = 2*P_0 - P_1, mirrored at the tail), which
 * makes the two-point path exactly the straight segment between them.
 *
 * Arc length is tabulated at construction (kArcSamplesPerSpan uniform parameter samples per span,
 * chord-accumulated); positionAt()/spanFractionAt() resolve a path length s through that table
 * with linear interpolation. minCurvatureRadius() is estimated from circumradii of consecutive
 * sample triples during the same pass and drives the block's curvature speed cap (REQ-SPL-06).
 *
 * Constructors must not throw (project mandate): degenerate input (fewer than two points, or
 * consecutive points closer than kMinPointSeparation) leaves the path INVALID with a typed
 * geometry error; callers must check isValid() before use.
 */
class SplinePath {
public:
    /** @brief Geometric reasons an interpolating spline cannot be constructed. */
    enum class GeometryError {
        None,             ///< Geometry is valid.
        TooFewPoints,     ///< Fewer than two interpolation points.
        CoincidentPoints  ///< Two consecutive points are closer than kMinPointSeparation.
    };

    /// Minimum distance between consecutive interpolation points for a well-conditioned span
    /// (same policy value as CircMotionProfile::kMinPointSeparation).
    static constexpr Millimeters kMinPointSeparation{0.01};
    /// Uniform parameter samples per span for the arc-length table and curvature estimation.
    /// 32 chords bound the length error of a well-behaved span to well below the 0.01 mm
    /// point-separation floor at typical teach distances.
    static constexpr int kArcSamplesPerSpan = 32;

    /** @brief Builds the path through @p points_mm (positions in millimeters, in travel order). */
    explicit SplinePath(const std::vector<Eigen::Vector3d>& points_mm);

    [[nodiscard]] bool isValid() const { return geometry_error_ == GeometryError::None; }
    [[nodiscard]] GeometryError geometryError() const { return geometry_error_; }
    /** @brief Human-readable name of the geometry defect for diagnostics/logging. */
    [[nodiscard]] static const char* geometryErrorName(GeometryError error);

    /** @brief Total arc length of the path [mm]. Zero when invalid. */
    [[nodiscard]] double totalLength() const { return total_length_mm_; }

    /** @brief Number of spans (interpolation points - 1). Zero when invalid. */
    [[nodiscard]] int spanCount() const;

    /**
     * @brief Minimum curvature radius found along the sampled curve [mm].
     * Returns a very large value (>= kStraightCurvatureRadius) for a straight path.
     */
    [[nodiscard]] double minCurvatureRadius() const { return min_curvature_radius_mm_; }

    /// Curvature radius reported for straight (zero-curvature) sample triples [mm].
    static constexpr double kStraightCurvatureRadius = 1.0e12;

    /** @brief Position on the curve at path length @p s_mm, clamped to [0, totalLength()]. */
    [[nodiscard]] Eigen::Vector3d positionAt(double s_mm) const;

    /** @brief Location of path length s within the span structure (drives per-span orientation slerp). */
    struct SpanLocation {
        int span = 0;          ///< Span index in [0, spanCount()-1].
        double fraction = 0.0; ///< Normalized arc-length fraction within that span, in [0, 1].
    };
    [[nodiscard]] SpanLocation spanLocationAt(double s_mm) const;

private:
    // Evaluates the centripetal Catmull-Rom span @p span at local parameter t in [t1, t2]
    // (Barry-Goldman pyramid over the four control points of the span).
    [[nodiscard]] Eigen::Vector3d evaluateSpan(int span, double t) const;

    GeometryError geometry_error_ = GeometryError::TooFewPoints;
    std::vector<Eigen::Vector3d> points_;       ///< Interpolation points incl. phantom endpoints.
    std::vector<double> knots_;                 ///< Centripetal knot value per point in points_.
    // Per-span arc-length table: sample_s_[span][k] is the cumulative PATH length at uniform
    // parameter sample k of that span; sample_t_[span][k] the matching parameter value.
    std::vector<std::vector<double>> sample_s_;
    std::vector<std::vector<double>> sample_t_;
    std::vector<double> span_end_s_;            ///< Cumulative path length at each span end.
    double total_length_mm_ = 0.0;
    double min_curvature_radius_mm_ = kStraightCurvatureRadius;
};

} // namespace RDT
