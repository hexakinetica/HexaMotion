// FrameTransformer.cpp
#include "FrameTransformer.h"
#include <kdl/frames.hpp>

namespace RDT {

// Helper for conversion as Units.h seems to lack these methods
static inline double deg2rad(double deg) { return deg * UnitConstants::PI / 180.0; }
static inline double rad2deg(double rad) { return rad * 180.0 / UnitConstants::PI; }
static inline double mm2m(double mm) { return mm / 1000.0; }
static inline double m2mm(double m) { return m * 1000.0; }

// --- Internal Helper Functions ---

/**
 * @internal
 * @brief Converts an RDT::CartPose to a KDL::Frame.
 * Handles the conversion from Degrees/Millimeters to Radians/Meters.
 */
static KDL::Frame toKdlFrame(const CartPose& pose) {
    return KDL::Frame(
        KDL::Rotation::RPY(deg2rad(pose.rx.value()), deg2rad(pose.ry.value()), deg2rad(pose.rz.value())),
        KDL::Vector(mm2m(pose.x.value()), mm2m(pose.y.value()), mm2m(pose.z.value()))
    );
}

/**
 * @internal
 * @brief Converts a KDL::Frame back to an RDT::CartPose.
 * Handles the conversion from Radians/Meters to Degrees/Millimeters.
 */
static CartPose fromKdlFrame(const KDL::Frame& frame) {
    CartPose result;
    result.x = Millimeters(m2mm(frame.p.x()));
    result.y = Millimeters(m2mm(frame.p.y()));
    result.z = Millimeters(m2mm(frame.p.z()));

    double r, p, y;
    frame.M.GetRPY(r, p, y);
    result.rx = Degrees(rad2deg(r));
    result.ry = Degrees(rad2deg(p));
    result.rz = Degrees(rad2deg(y));
    return result;
}

// --- Public Static Methods ---

CartPose FrameTransformer::calculateTcpInWorld(const CartPose& flange_pose, const CartPose& tool_transform) {
    return fromKdlFrame(toKdlFrame(flange_pose) * toKdlFrame(tool_transform));
}

CartPose FrameTransformer::calculateFlangeInWorld(const CartPose& tcp_pose, const CartPose& tool_transform) {
    return fromKdlFrame(toKdlFrame(tcp_pose) * toKdlFrame(tool_transform).Inverse());
}

CartPose FrameTransformer::applyBaseTransform(const CartPose& base_transform, const CartPose& pose) {
    return fromKdlFrame(toKdlFrame(base_transform) * toKdlFrame(pose));
}

} // namespace RDT
