// MotionSegment.cpp
#include "MotionSegment.h"

namespace RDT {

MotionSegment::MotionSegment(std::vector<TrajectoryPoint>&& points, std::shared_ptr<MotionProfile> profile)
    : points_(std::move(points)), profile_(profile) {}

const std::vector<TrajectoryPoint>& MotionSegment::getPoints() const {
    return points_;
}

Seconds MotionSegment::getDuration() const {
    return profile_ ? profile_->getDuration() : 0.0_s;
}

MotionType MotionSegment::getMotionType() const {
    return profile_ ? profile_->getMotionType() : MotionType::HOLD;
}

bool MotionSegment::isEmpty() const {
    return points_.empty();
}

} // namespace RDT