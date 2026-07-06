// Trajectory.cpp
#include "Trajectory.h"

namespace RDT {

void Trajectory::addSegment(std::unique_ptr<MotionSegment> segment) {
    if (!segment || segment->isEmpty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    segments_.push_back(std::move(segment));
}

std::vector<TrajectoryPoint> Trajectory::getNextPointBatch(size_t batch_size) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<TrajectoryPoint> batch;
    if (segments_.empty()) {
        return batch;
    }
    
    batch.reserve(batch_size);

    while (batch.size() < batch_size && !segments_.empty()) {
        const auto& current_segment = segments_.front();
        const auto& points = current_segment->getPoints();

        size_t points_to_copy = std::min(batch_size - batch.size(), points.size() - current_point_index_);
        
        if (points_to_copy > 0) {
            batch.insert(batch.end(),
                         points.begin() + current_point_index_,
                         points.begin() + current_point_index_ + points_to_copy);
            current_point_index_ += points_to_copy;
        }

        // If current segment is finished, move to the next one
        if (current_point_index_ >= points.size()) {
            segments_.pop_front();
            current_point_index_ = 0;
        }
    }

    return batch;
}

void Trajectory::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    segments_.clear();
    current_point_index_ = 0;
}

bool Trajectory::isFinished() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return segments_.empty();
}

} // namespace RDT