// MotionManager.cpp
#include "MotionManager.h"
#include <stdexcept>
#include <chrono>
#include <cmath>

namespace RDT {

using namespace std::chrono_literals;

MotionManager::MotionManager(std::shared_ptr<HardwareManager> hw_manager,
                             unsigned int cycle_period_ms,
                             const RobotLimits& limits,
                             Degrees following_error_threshold)
    : hw_manager_(hw_manager),
      cycle_period_(cycle_period_ms),
      limits_(limits),
      following_error_threshold_(following_error_threshold) {

    if (!hw_manager_) {
        // This is a programming error, not a runtime error. Crashing is appropriate.
        RDT_LOG_CRITICAL(MODULE_NAME, "HardwareManager cannot be null.");
        throw std::invalid_argument("MotionManager: HardwareManager cannot be null.");
    }
    if (cycle_period_ms == 0) {
        RDT_LOG_CRITICAL(MODULE_NAME, "Cycle period cannot be zero.");
        throw std::invalid_argument("MotionManager: Invalid cycle period.");
    }
    RDT_LOG_INFO(MODULE_NAME, "MotionManager created. Cycle: {}ms, Following Error Threshold: {}.",
        cycle_period_ms, following_error_threshold_.toString());
}

MotionManager::~MotionManager() {
    stop();
}

bool MotionManager::start() {
    if (running_.load()) {
        RDT_LOG_WARN(MODULE_NAME, "RT Loop is already running.");
        return true;
    }

    // Read initial state to populate last_sent_joints_
    auto initial_fb_res = hw_manager_->read();
    if (initial_fb_res.isError()) {
        RDT_LOG_ERROR(MODULE_NAME, "Hardware manager failed to provide initial state. Cannot start RT loop.");
        current_state_.store(RTState::Error);
        return false;
    }
    last_sent_joints_ = initial_fb_res.value().joints;

    RDT_LOG_INFO(MODULE_NAME, "Starting RT Loop...");
    running_ = true;
    current_state_.store(RTState::Idle);
    rt_thread_ = std::jthread(&MotionManager::rt_cycle_tick, this, rt_thread_.get_stop_token());
    return true;
}

void MotionManager::stop() {
    if (running_.exchange(false)) {
        RDT_LOG_INFO(MODULE_NAME, "Stopping RT Loop...");
        if (rt_thread_.joinable()) {
            rt_thread_.request_stop();
            rt_thread_.join();
            RDT_LOG_INFO(MODULE_NAME, "RT Loop stopped and thread joined.");
        }
    }
}

void MotionManager::emergencyStop() {
    RDT_LOG_CRITICAL(MODULE_NAME, "Emergency Stop requested! Clearing all command queues.");
    current_state_.store(RTState::Error);
    command_queue_.clear();
    rt_command_buffer_.clear();
}

void MotionManager::reset() {
    RDT_LOG_INFO(MODULE_NAME, "Reset requested.");
    command_queue_.clear();
    rt_command_buffer_.clear();
    current_state_.store(RTState::Idle);
}

bool MotionManager::enqueueCommand(const TrajectoryPoint& cmd_point) {
    if (!command_queue_.try_push(cmd_point)) {
        RDT_LOG_WARN(MODULE_NAME, "Main command queue overflow!");
        return false;
    }
    return true;
}

bool MotionManager::dequeueFeedback(TrajectoryPoint& out_point) {
    auto res = feedback_queue_.try_pop();
    if (res.isSuccess()) {
        out_point = std::move(res.value());
        return true;
    }
    return false;
}

size_t MotionManager::getCommandQueueSize() const { return command_queue_.size(); }
size_t MotionManager::getFeedbackQueueSize() const { return feedback_queue_.size(); }
RTState MotionManager::getCurrentState() const { return current_state_.load(); }

// --- Private Methods ---

void MotionManager::rt_cycle_tick(std::stop_token stoken) {
    RDT_LOG_INFO(MODULE_NAME, "RT thread started.");
    
    while (!stoken.stop_requested() && running_.load()) {
        auto cycle_start_time = std::chrono::steady_clock::now();

        // This will be the main data packet sent upstream this cycle
        TrajectoryPoint feedback_packet{};
        
        // 1. Service queues and determine target if not in error state
        if (current_state_.load() != RTState::Error) {
            serviceMainCommandQueue(feedback_packet);

            if (!rt_command_buffer_.empty()) {
                current_state_.store(RTState::Moving);
                last_sent_joints_ = rt_command_buffer_.front().command.joint_target;
                feedback_packet.header = rt_command_buffer_.front().header; // Preserve metadata
                rt_command_buffer_.pop_front();
            } else {
                if (current_state_.load() == RTState::Moving) {
                    current_state_.store(RTState::Idle); // Transition to Idle once buffer is empty
                }
                // While idle, command is to hold the last sent position
                feedback_packet.header.motion_type = MotionType::HOLD; // Explicitly state that we are holding position
            }
        }
        feedback_packet.command.joint_target = last_sent_joints_;

        // 2. Send command to hardware
        auto write_res = hw_manager_->write(last_sent_joints_);
        if (write_res.isError()) {
            RDT_LOG_CRITICAL(MODULE_NAME, "HAL write failed: {}. Entering ERROR state.", ToString(write_res.error()));
            current_state_.store(RTState::Error);
            feedback_packet.diagnostics.hal = HalStatus::Error_CommunicationLost;
        }

        // 3. Read feedback from hardware
        auto read_res = hw_manager_->read();
        if (read_res.isError()) {
            RDT_LOG_CRITICAL(MODULE_NAME, "HAL read failed: {}. Entering ERROR state.", ToString(read_res.error()));
            current_state_.store(RTState::Error);
            feedback_packet.diagnostics.hal = HalStatus::Error_CommunicationLost;
        } else {
            const auto& hw_fb = read_res.value();
            feedback_packet.feedback.joint_actual = hw_fb.joints;
            feedback_packet.diagnostics.hal = hw_fb.driver_status;

            // 4. Perform safety checks
            checkFollowingError(last_sent_joints_, hw_fb.joints, feedback_packet);
            
            // If E-Stop is active from HAL, force error state
            if (hw_fb.safety.is_estop_active) {
                current_state_.store(RTState::Error);
                feedback_packet.diagnostics.safety = SafetyStatus::Error_EStop_Active;
            }
        }
        
        // 5. Finalize and push feedback packet
        feedback_packet.feedback.rt_state = current_state_.load();
        if (!feedback_queue_.try_push(feedback_packet)) {
            // If feedback queue is full, drop the oldest message to make space
            TrajectoryPoint dummy;
            feedback_queue_.try_pop();
            feedback_queue_.try_push(feedback_packet);
        }

        sleepUntilNextCycle(cycle_start_time);
    }
    RDT_LOG_INFO(MODULE_NAME, "RT thread finishing.");
}

void MotionManager::serviceMainCommandQueue(TrajectoryPoint& current_feedback_packet) {
    // Refill the RT buffer from the main NRT queue if there is space
    while (rt_command_buffer_.size() < RT_BUFFER_REFILL_THRESHOLD) {
        auto res = command_queue_.try_pop();
        if (res.isSuccess()) {
            TrajectoryPoint new_target = res.value();
            if (validateTargetPoint(new_target)) {
                rt_command_buffer_.push_back(new_target);
            } else {
                // Validation failed, error state is set inside validateTargetPoint.
                // Copy the diagnostics to the current feedback packet so the NRT world sees it.
                current_feedback_packet.diagnostics = new_target.diagnostics;
                current_feedback_packet.header = new_target.header;

                // Clear all queues to stop the invalid motion.
                rt_command_buffer_.clear();
                command_queue_.clear();
                break; // Stop trying to fill the buffer
            }
        } else {
            break; // Main queue is empty, nothing more to do
        }
    }
}

bool MotionManager::validateTargetPoint(TrajectoryPoint& point) {
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        const auto& limit_pair_deg = limits_.joint_position_limits_deg[i];
        AxisId axis = static_cast<AxisId>(i);
        Degrees target_angle_deg = point.command.joint_target[axis].position;
        
        if (target_angle_deg < limit_pair_deg.first || target_angle_deg > limit_pair_deg.second) {
            RDT_LOG_ERROR(MODULE_NAME, "Target for Axis {} ({}) is outside position limits [{}, {}]! Motion stopped.",
                        (i + 1), target_angle_deg.toString(), limit_pair_deg.first.toString(), limit_pair_deg.second.toString());
            
            point.diagnostics.safety = SafetyStatus::Error_JointLimit;
            point.diagnostics.failing_axis_id = static_cast<int8_t>(i);
            current_state_.store(RTState::Error);
            return false;
        }
    }
    return true;
}

bool MotionManager::checkFollowingError(const AxisSet& last_cmd, const AxisSet& current_fb, TrajectoryPoint& out_point) {
    if (current_state_.load() == RTState::Moving) { // Only check when actively moving
        for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            auto cmd_pos = last_cmd.GetAt(i).value().get().position;
            auto fb_pos = current_fb.GetAt(i).value().get().position;
            Degrees error = (cmd_pos - fb_pos).abs();

            if (error > following_error_threshold_) {
                RDT_LOG_CRITICAL(MODULE_NAME, "Following Error on Axis {}! Error: {}, Limit: {}. Entering ERROR state.",
                    (i + 1), error.toString(), following_error_threshold_.toString());
                
                out_point.diagnostics.safety = SafetyStatus::Error_FollowingError;
                out_point.diagnostics.failing_axis_id = static_cast<int8_t>(i);
                current_state_.store(RTState::Error);
                return false;
            }
        }
    }
    return true;
}


void MotionManager::sleepUntilNextCycle(const std::chrono::steady_clock::time_point& cycle_start_time) const {
    std::this_thread::sleep_until(cycle_start_time + cycle_period_);
}

} // namespace RDT
