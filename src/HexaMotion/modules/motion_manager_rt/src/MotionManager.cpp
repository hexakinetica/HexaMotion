// MotionManager.cpp
#include "MotionManager.h"
#include <chrono>
#include <cmath>

#ifdef _WIN32
// High-resolution timer request for the RT thread (REQ-RT-PACE-01). Without it, sleep_until wakes
// on the default ~15.6 ms Windows timer and the nominal 4 ms cycle paces ~3.9x slower (measured).
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <timeapi.h>
#endif

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
        RDT_LOG_CRITICAL(MODULE_NAME, "HardwareManager cannot be null.");
        dependencies_valid_ = false;
    }
    if (cycle_period_ms == 0) {
        RDT_LOG_CRITICAL(MODULE_NAME, "Cycle period cannot be zero.");
        dependencies_valid_ = false;
    }
    RDT_LOG_INFO(MODULE_NAME, "MotionManager created. Cycle: {}ms, Following Error Threshold: {}.",
        cycle_period_ms, following_error_threshold_.toString());
}

MotionManager::~MotionManager() {
    stop();
}

bool MotionManager::start() {
    if (!dependencies_valid_) {
        current_state_.store(RTState::Error);
        return false;
    }
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
    // Halt motion immediately: Error state makes the RT loop hold position and skip command
    // processing on its next tick. These are atomics, safe to set from any (NRT) thread.
    current_state_.store(RTState::Error);
    hal_motion_active_.store(false, std::memory_order_release);
    if (running_.load()) {
        // command_queue_ and especially rt_command_buffer_ (an RT-thread-only ring) are owned by the RT
        // thread; clearing them from here races with the running RT loop. Ask the RT thread to clear
        // them on its next tick. Fire-and-forget: an E-Stop must never block on a handshake.
        estop_request_.store(true, std::memory_order_release);
    } else {
        // RT thread not running: no concurrent access, safe to clear directly.
        command_queue_.clear();
        rt_command_buffer_.clear();
        waiting_for_segment_completion_ = false;
    }
}

void MotionManager::reset() {
    RDT_LOG_INFO(MODULE_NAME, "Reset requested.");
    if (running_.load()) {
        reset_ack_.store(false);
        reset_request_.store(true);
        
        auto start_time = std::chrono::steady_clock::now();
        while (!reset_ack_.load(std::memory_order_acquire)) {
            if (!running_.load()) break; // Thread stopped unexpectedly
            
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() > 500) {
                 RDT_LOG_ERROR(MODULE_NAME, "Reset timed out waiting for RT thread!");
                 break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    } else {
        command_queue_.clear();
        rt_command_buffer_.clear();
        waiting_for_segment_completion_ = false;
        hal_motion_active_.store(false, std::memory_order_release);
        current_state_.store(RTState::Idle);
    }
}

bool MotionManager::enqueueCommand(const TrajectoryPoint& cmd_point) {
    if (!dependencies_valid_) {
        return false;
    }
    if (!command_queue_.try_push(cmd_point)) {
        RDT_LOG_WARN(MODULE_NAME, "Main command queue overflow!");
        return false;
    }
    return true;
}

bool MotionManager::dequeueFeedback(TrajectoryPoint& out_point) {
    if (!dependencies_valid_) {
        return false;
    }
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!feedback_fresh_) {
        return false;
    }
    out_point = feedback_value_;
    feedback_fresh_ = false;
    return true;
}

size_t MotionManager::getCommandQueueSize() const { return command_queue_.size(); }
size_t MotionManager::getFeedbackQueueSize() const {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return feedback_fresh_ ? 1u : 0u;
}
RTState MotionManager::getCurrentState() const { return current_state_.load(); }

// --- Private Methods ---

void MotionManager::rt_cycle_tick(std::stop_token stoken) {
    RDT_LOG_INFO(MODULE_NAME, "RT thread started.");

#ifdef _WIN32
    // REQ-RT-PACE-01: request 1 ms system timer resolution for the lifetime of the RT thread.
    // Without it, sleep_until wakes on the default ~15.6 ms Windows timer, so the configured
    // cycle (4 ms) ran at ~15.6 ms and ALL streamed motion executed ~3.9x slower than planned
    // (one pre-rendered point is consumed per tick). Paired with timeEndPeriod() at thread exit.
    const bool high_res_timer_ok = (timeBeginPeriod(1) == TIMERR_NOERROR);
    if (!high_res_timer_ok) {
        RDT_LOG_WARN(MODULE_NAME,
            "timeBeginPeriod(1) failed: the OS keeps the coarse (~15.6 ms) sleep granularity, so "
            "the {} ms RT cycle will pace slower than configured. Motion stays geometrically "
            "correct but runs proportionally slower; check the OS timer policy.",
            cycle_period_.count());
    }
#endif

    // REQ-RT-PACE-02: absolute cycle schedule. The deadline advances by exactly one period per
    // cycle, so a late wake is compensated by a shorter next sleep and the long-run average holds
    // the configured rate. The previous pattern (re-sampling "now" after every wake) added the
    // wake-up overshoot to every period, with no schedule to catch up to.
    auto next_cycle_time = std::chrono::steady_clock::now() + cycle_period_;
    // REQ-RT-PACE-03: stall guard. After a long stall (OS preemption, debugger, blocking HAL call)
    // do not burst the missed cycles back-to-back — each tick streams one trajectory point, so a
    // burst would step the commanded position by several points at once. Beyond this lag the
    // schedule is resynced and the lost time is dropped: motion is briefly slower, never a jump.
    const auto max_schedule_lag = cycle_period_ * 4;
    auto last_resync_log = std::chrono::steady_clock::time_point{};

    while (!stoken.stop_requested() && running_.load()) {
        if (reset_request_.exchange(false)) {
            command_queue_.clear();
            rt_command_buffer_.clear();
            waiting_for_segment_completion_ = false;
            hal_motion_active_.store(false, std::memory_order_release);
            current_state_.store(RTState::Idle);
            error_latched_ = false; // leaving the error condition: stop re-publishing the latched fault
            
            // Sync command to current feedback to prevent jumps after mode switch
            auto read_res = hw_manager_->read();
            if (read_res.isSuccess()) {
                last_sent_joints_ = read_res.value().joints;
            }

            segment_end_hold_point_ = {};

            reset_ack_.store(true, std::memory_order_release);
            RDT_LOG_INFO(MODULE_NAME, "RT Loop reset complete.");
        }

        if (estop_request_.exchange(false)) {
            // E-Stop requested from the NRT world: clear the RT-owned buffers here (never from the
            // caller). State was already forced to Error by emergencyStop(); keep it until reset.
            command_queue_.clear();
            rt_command_buffer_.clear();
            waiting_for_segment_completion_ = false;
            hal_motion_active_.store(false, std::memory_order_release);
        }

        // This will be the main data packet sent upstream this cycle
        TrajectoryPoint feedback_packet{};
        HardwareCommand hw_command{};
        hw_command.stream_target = last_sent_joints_;
        
        // 1. Service queues and determine target if not in error state
        if (current_state_.load() != RTState::Error) {
            serviceMainCommandQueue(feedback_packet);

            if (waiting_for_segment_completion_) {
                feedback_packet = segment_end_hold_point_;
                last_sent_joints_ = segment_end_hold_point_.command.joint_target;
                hw_command.segment_target = segment_end_hold_point_.segment_target;
                hw_command.has_segment_target = true;
                if (current_state_.load() != RTState::Error) {
                    current_state_.store(RTState::Moving);
                }
            } else if (!rt_command_buffer_.empty()) {
                const TrajectoryPoint active_point = rt_command_buffer_.front();
                rt_command_buffer_.pop_front();

                last_sent_joints_ = active_point.command.joint_target;
                feedback_packet = active_point;
                hw_command.segment_target = active_point.segment_target;
                hw_command.has_segment_target = true;

                if (active_point.header.is_target_reached_for_this_point) {
                    segment_end_hold_point_ = active_point;
                    waiting_for_segment_completion_ = true;
                }

                if (current_state_.load() != RTState::Error) {
                    current_state_.store(RTState::Moving);
                }
            } else {
                if (current_state_.load() == RTState::Moving && !hal_motion_active_.load(std::memory_order_acquire)) {
                    current_state_.store(RTState::Idle);
                }
                feedback_packet.header.motion_type = MotionType::HOLD;
            }
        }

        hw_command.stream_target = last_sent_joints_;
        feedback_packet.command.joint_target = last_sent_joints_;

        // 2. Send command to hardware
        auto write_res = hw_manager_->write(hw_command);
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
            feedback_packet.feedback.target_reached = hw_fb.target_reached;
            // Ride the raw DI bitmask on the circulating feedback so the NRT program sequencer reads IO
            // from the same coherent snapshot it uses for motion completion (one sample instant).
            feedback_packet.feedback.digital_inputs = hw_fb.digital_inputs;
            feedback_packet.diagnostics.hal = hw_fb.driver_status;
            hal_motion_active_.store(hw_fb.motion_active, std::memory_order_release);

            // 4. Perform safety checks
            if (isHomingActive()) {
                // During native HAL homing the axes are allowed to move independently from
                // the RT hold target. Treat the hardware motion as authoritative and resync
                // the hold command so we do not trip a false following error.
                last_sent_joints_ = hw_fb.joints;
                waiting_for_segment_completion_ = false;
                if (current_state_.load() != RTState::Error) {
                    current_state_.store(RTState::Idle);
                }
            } else {
                if (hw_fb.motion_active) {
                    if (current_state_.load() != RTState::Error) {
                        current_state_.store(RTState::Moving);
                    }
                } else {
                    if (waiting_for_segment_completion_ && hw_fb.target_reached) {
                        waiting_for_segment_completion_ = false;
                    }

                    // Following-error is only meaningful when HexaMotion streams per-cycle
                    // setpoints (Teleport). When the drive runs its own profile to the
                    // commanded endpoint (InternalInterpolator, e.g. the MKS firmware), the
                    // commanded endpoint legitimately leads the lagging actual during a move,
                    // so this check would falsely trip Error_FollowingError (notably on far
                    // jogs). Skip it in that mode.
                    if (hw_fb.execution_mode != HalCommandExecutionMode::InternalInterpolator) {
                        checkFollowingError(last_sent_joints_, hw_fb.joints, feedback_packet);
                    }

                    if (current_state_.load() != RTState::Error &&
                        rt_command_buffer_.empty() &&
                        !waiting_for_segment_completion_) {
                        current_state_.store(RTState::Idle);
                    }
                }
            }
            
            // If E-Stop is active from HAL, force error state
            if (hw_fb.safety.is_estop_active) {
                current_state_.store(RTState::Error);
                feedback_packet.diagnostics.safety = SafetyStatus::Error_EStop_Active;
            }
        }
        
        // 5. Finalize and publish the latest feedback packet. Overwriting the previous unread sample
        // is intentional (drop-oldest): the NRT consumer only needs the freshest robot state.
        feedback_packet.feedback.rt_state = current_state_.load();
        // Latch the fault diagnostic so the latest-value mailbox cannot drop it: in Error state the
        // loop skips processing, so the causing diagnostic would otherwise live for only one cycle.
        if (feedback_packet.feedback.rt_state == RTState::Error) {
            if (!error_latched_) {
                latched_diagnostics_ = feedback_packet.diagnostics;
                error_latched_ = true;
            } else {
                feedback_packet.diagnostics = latched_diagnostics_;
            }
        }
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            feedback_value_ = feedback_packet;
            feedback_fresh_ = true;
        }

        std::this_thread::sleep_until(next_cycle_time);
        next_cycle_time += cycle_period_;
        const auto wake_time = std::chrono::steady_clock::now();
        if (wake_time - next_cycle_time > max_schedule_lag) {
            if (wake_time - last_resync_log > std::chrono::seconds(5)) {
                RDT_LOG_WARN(MODULE_NAME,
                    "RT cycle overran its schedule by {} ms; schedule resynced (missed cycles are "
                    "dropped, not burst).",
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        wake_time - next_cycle_time).count());
                last_resync_log = wake_time;
            }
            next_cycle_time = wake_time + cycle_period_;
        }
    }

#ifdef _WIN32
    if (high_res_timer_ok) {
        timeEndPeriod(1);
    }
#endif
    RDT_LOG_INFO(MODULE_NAME, "RT thread finishing.");
}

void MotionManager::serviceMainCommandQueue(TrajectoryPoint& current_feedback_packet) {
    // Refill the RT buffer from the main NRT queue if there is space
    while (rt_command_buffer_.size() < RT_BUFFER_REFILL_THRESHOLD) {
        auto res = command_queue_.try_pop();
        if (res.isSuccess()) {
            TrajectoryPoint new_target = res.value();
            if (validateTargetPoint(new_target)) {
                if (!rt_command_buffer_.push_back(new_target)) {
                    // Unreachable while the refill invariant holds (size() < threshold < ring
                    // capacity). Fail visible rather than dropping a motion point silently.
                    RDT_LOG_ERROR(MODULE_NAME,
                        "RT command ring is full ({} points) during refill - a motion point could "
                        "not be buffered. Refill invariant broken; motion continues from the "
                        "already-buffered points.",
                        rt_command_buffer_.size());
                    break;
                }
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

bool MotionManager::isHomingActive() const {
    // Homing is owned by the backend; HexaMotion does not track a homing state machine.
    // We only need to know whether any axis is *currently* homing so the RT loop treats the
    // hardware motion as authoritative and does not trip a false following error. That is
    // read directly from the raw axis state reported by the backend (shared AxisState
    // convention: Homing=5, AutoHoming=6, HomingOffset=7).
    constexpr int kRawStateHoming = 5;
    constexpr int kRawStateAutoHoming = 6;
    constexpr int kRawStateHomingOffset = 7;
    const auto hal_state = hw_manager_->getCurrentHalState();
    for (const auto& axis : hal_state.axes) {
        if (axis.last_status == kRawStateHoming ||
            axis.last_status == kRawStateAutoHoming ||
            axis.last_status == kRawStateHomingOffset) {
            return true;
        }
    }
    return false;
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

} // namespace RDT
