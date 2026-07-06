// SplinePath.cpp
#include "SplinePath.h"

#include <algorithm>
#include <cmath>

namespace RDT {

namespace {
// Circumradius of the triangle (a, b, c) [mm]. Near-collinear triples have a vanishing cross
// product and would divide by ~0, so they report SplinePath::kStraightCurvatureRadius instead
// (a straight sample triple means locally zero curvature — no cap needed there).
double circumradius(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c) {
    const Eigen::Vector3d ab = b - a;
    const Eigen::Vector3d ac = c - a;
    const Eigen::Vector3d bc = c - b;
    const double cross_norm = ab.cross(ac).norm();
    constexpr double kMinCrossNorm = 1.0e-9; // [mm^2]; below this the triple is numerically straight
    if (cross_norm < kMinCrossNorm) {
        return SplinePath::kStraightCurvatureRadius;
    }
    return (ab.norm() * ac.norm() * bc.norm()) / (2.0 * cross_norm);
}
} // namespace

SplinePath::SplinePath(const std::vector<Eigen::Vector3d>& points_mm) {
    if (points_mm.size() < 2) {
        geometry_error_ = GeometryError::TooFewPoints;
        return;
    }
    for (size_t i = 1; i < points_mm.size(); ++i) {
        if ((points_mm[i] - points_mm[i - 1]).norm() < kMinPointSeparation.value()) {
            geometry_error_ = GeometryError::CoincidentPoints;
            return;
        }
    }

    // Control-point list with phantom endpoints so every span has four control points. The
    // reflection keeps the end tangents pointing along the first/last chord, which makes the
    // two-point spline exactly its straight segment (REQ-SPL-09 single-point block semantics).
    points_.reserve(points_mm.size() + 2);
    points_.push_back(2.0 * points_mm.front() - points_mm[1]);
    points_.insert(points_.end(), points_mm.begin(), points_mm.end());
    points_.push_back(2.0 * points_mm.back() - points_mm[points_mm.size() - 2]);

    // Centripetal knots: t_{i+1} = t_i + |P_{i+1} - P_i|^(1/2). The coincidence guard above bounds
    // every increment away from zero, so no knot interval can collapse.
    knots_.resize(points_.size());
    knots_[0] = 0.0;
    for (size_t i = 1; i < points_.size(); ++i) {
        knots_[i] = knots_[i - 1] + std::sqrt((points_[i] - points_[i - 1]).norm());
    }

    // Arc-length tables per span (span i interpolates points_[i+1] .. points_[i+2]).
    const int spans = static_cast<int>(points_mm.size()) - 1;
    sample_s_.resize(spans);
    sample_t_.resize(spans);
    span_end_s_.resize(spans);
    double s_accum = 0.0;
    Eigen::Vector3d prev_prev = points_mm.front(); // for curvature triples across sample steps
    Eigen::Vector3d prev = points_mm.front();
    bool have_two_samples = false;

    for (int span = 0; span < spans; ++span) {
        const double t1 = knots_[span + 1];
        const double t2 = knots_[span + 2];
        sample_s_[span].resize(kArcSamplesPerSpan + 1);
        sample_t_[span].resize(kArcSamplesPerSpan + 1);
        for (int k = 0; k <= kArcSamplesPerSpan; ++k) {
            const double t = t1 + (t2 - t1) * static_cast<double>(k) / kArcSamplesPerSpan;
            const Eigen::Vector3d p = evaluateSpan(span, t);
            if (!(span == 0 && k == 0)) { // first sample of the whole path has no predecessor
                s_accum += (p - prev).norm();
                if (have_two_samples) {
                    min_curvature_radius_mm_ =
                        std::min(min_curvature_radius_mm_, circumradius(prev_prev, prev, p));
                }
                prev_prev = prev;
                have_two_samples = true;
            }
            prev = p;
            sample_s_[span][k] = s_accum;
            sample_t_[span][k] = t;
        }
        span_end_s_[span] = s_accum;
    }
    total_length_mm_ = s_accum;
    geometry_error_ = GeometryError::None;
}

const char* SplinePath::geometryErrorName(GeometryError error) {
    switch (error) {
        case GeometryError::None: return "None";
        case GeometryError::TooFewPoints: return "TooFewPoints (a spline needs at least two points)";
        case GeometryError::CoincidentPoints:
            return "CoincidentPoints (consecutive points closer than the minimum separation)";
    }
    return "UnknownGeometryError";
}

int SplinePath::spanCount() const {
    return isValid() ? static_cast<int>(span_end_s_.size()) : 0;
}

Eigen::Vector3d SplinePath::evaluateSpan(int span, double t) const {
    // Barry-Goldman pyramid over the four control points of the span. Indices into points_/knots_:
    // span i uses points_[i .. i+3] with the curve segment between knots_[i+1] and knots_[i+2].
    const Eigen::Vector3d& p0 = points_[static_cast<size_t>(span)];
    const Eigen::Vector3d& p1 = points_[static_cast<size_t>(span) + 1];
    const Eigen::Vector3d& p2 = points_[static_cast<size_t>(span) + 2];
    const Eigen::Vector3d& p3 = points_[static_cast<size_t>(span) + 3];
    const double t0 = knots_[static_cast<size_t>(span)];
    const double t1 = knots_[static_cast<size_t>(span) + 1];
    const double t2 = knots_[static_cast<size_t>(span) + 2];
    const double t3 = knots_[static_cast<size_t>(span) + 3];

    const Eigen::Vector3d a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1;
    const Eigen::Vector3d a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2;
    const Eigen::Vector3d a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3;
    const Eigen::Vector3d b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2;
    const Eigen::Vector3d b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3;
    return (t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2;
}

SplinePath::SpanLocation SplinePath::spanLocationAt(double s_mm) const {
    SpanLocation loc;
    if (!isValid()) {
        return loc;
    }
    const double s = std::clamp(s_mm, 0.0, total_length_mm_);
    // Find the span whose cumulative end length covers s.
    const auto span_it = std::lower_bound(span_end_s_.begin(), span_end_s_.end(), s);
    loc.span = static_cast<int>(std::distance(span_end_s_.begin(), span_it));
    if (loc.span >= spanCount()) {
        loc.span = spanCount() - 1;
    }
    const double span_start_s = (loc.span == 0) ? 0.0 : span_end_s_[static_cast<size_t>(loc.span) - 1];
    const double span_len = span_end_s_[static_cast<size_t>(loc.span)] - span_start_s;
    loc.fraction = (span_len > 0.0) ? std::clamp((s - span_start_s) / span_len, 0.0, 1.0) : 1.0;
    return loc;
}

Eigen::Vector3d SplinePath::positionAt(double s_mm) const {
    if (!isValid()) {
        return Eigen::Vector3d::Zero();
    }
    const double s = std::clamp(s_mm, 0.0, total_length_mm_);
    const SpanLocation loc = spanLocationAt(s);
    const std::vector<double>& table_s = sample_s_[static_cast<size_t>(loc.span)];
    const std::vector<double>& table_t = sample_t_[static_cast<size_t>(loc.span)];
    // Invert s -> t inside the span table (samples are monotonically increasing in s).
    const auto it = std::lower_bound(table_s.begin(), table_s.end(), s);
    if (it == table_s.begin()) {
        return evaluateSpan(loc.span, table_t.front());
    }
    if (it == table_s.end()) {
        return evaluateSpan(loc.span, table_t.back());
    }
    const size_t hi = static_cast<size_t>(std::distance(table_s.begin(), it));
    const size_t lo = hi - 1;
    const double seg = table_s[hi] - table_s[lo];
    const double alpha = (seg > 0.0) ? (s - table_s[lo]) / seg : 1.0;
    const double t = table_t[lo] + alpha * (table_t[hi] - table_t[lo]);
    return evaluateSpan(loc.span, t);
}

} // namespace RDT
