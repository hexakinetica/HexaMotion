#include "HardwareManager.h"
#include "interface/IDriver.h"
#include "drivers/sim/SimDriver.h"
#include "drivers/udp/UdpDriver.h"
#include "drivers/udp/UdpPeer.hpp" 
#include <cmath>
#include <sstream>

namespace RDT {

HardwareManager::HardwareManager(const InterfaceConfig& config, const RobotLimits& limits)
    : config_(config)
    , limits_(limits)
{
    sim_driver_ = std::make_unique<SimDriver>(config.simulation_initial_joints);
}

HardwareManager::~HardwareManager() {
    shutdown();
}

Result<void, ErrorCode> HardwareManager::init() {
    auto sim_res = sim_driver_->init();
    if (sim_res.isError()) {
        return sim_res;
    }

    if (config_.realtime_type != InterfaceConfig::RealtimeInterfaceType::None) {
        if (!real_driver_) {
             if (config_.realtime_type == InterfaceConfig::RealtimeInterfaceType::Udp) {
                 real_driver_ = std::make_unique<UdpDriver>(config_.udp_control_config);
             } 
        }

        if (real_driver_) {
            auto real_res = real_driver_->init();
            if (real_res.isError()) {
                real_driver_.reset();
            }
        }
    }

    active_mode_.store(HalMode::Simulation, std::memory_order_relaxed);
    return Result<void, ErrorCode>::Success();
}

void HardwareManager::shutdown() {
    if (sim_driver_) sim_driver_->stop();
    if (real_driver_) real_driver_->stop();
}

Result<void, ErrorCode> HardwareManager::write(const AxisSet& cmd) {
    AxisSet safe_cmd = applyCommandGovernor(cmd);
    IDriver* active = getActiveDriverInstance();
    if (!active) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }
    return active->write(safe_cmd);
}

Result<HardwareFeedback, ErrorCode> HardwareManager::read() {
    IDriver* active = getActiveDriverInstance();
    if (!active) {
        return Result<HardwareFeedback, ErrorCode>::Failure(ErrorCode::NotConnected);
    }

    auto feedback_res = active->read();
    if (feedback_res.isError()) {
        return feedback_res;
    }
    HardwareFeedback final_feedback = feedback_res.value();

    if (active_mode_.load(std::memory_order_relaxed) == HalMode::Simulation && real_driver_) {
        auto real_fb_res = real_driver_->read();
        if (real_fb_res.isSuccess()) {
            final_feedback.safety = real_fb_res.value().safety;
        }
    }

    return Result<HardwareFeedback, ErrorCode>::Success(final_feedback);
}

Result<void, ErrorCode> HardwareManager::setMode(HalMode mode) {
    if (mode == active_mode_.load()) {
        return Result<void, ErrorCode>::Success();
    }

    if (mode == HalMode::Realtime) {
        if (!real_driver_) {
            return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
        }
        auto sim_res = sim_driver_->read();
        auto real_res = real_driver_->read();
        if (sim_res.isError() || real_res.isError()) {
            return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
        }
        const auto& sim_joints = sim_res.value().joints;
        const auto& real_joints = real_res.value().joints;
        constexpr double kSyncToleranceDeg = 1.0; 
        for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
             auto diff_deg = std::abs(sim_joints.GetAt(i).value().get().position.value() - real_joints.GetAt(i).value().get().position.value());
             if (diff_deg > kSyncToleranceDeg) {
                 return Result<void, ErrorCode>::Failure(ErrorCode::NetworkUnavailable);
             }
        }
    }

    active_mode_.store(mode);
    is_first_cmd_ = true;
    return Result<void, ErrorCode>::Success();
}

HalMode HardwareManager::getMode() const {
    return active_mode_.load(std::memory_order_relaxed);
}

void HardwareManager::syncSimulationToReal() {
    if (!real_driver_ || !sim_driver_) {
        return;
    }
    auto res = real_driver_->read();
    if (res.isSuccess()) {
        (void)sim_driver_->setState(res.value().joints);
    }
}

Result<void, ErrorCode> HardwareManager::zeroAxis(AxisId axis) {
    IDriver* target_driver = real_driver_ ? real_driver_.get() : sim_driver_.get();
    if (!target_driver) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }
    auto res = target_driver->resetAxisToZero(axis, 0.0);
    if (res.isSuccess()) {
        // Zeroing changes the coordinate system, invalidating the previous logical position state.
        // We must reset the governor to accept the new position as a fresh start point.
        is_first_cmd_ = true;
    }
    return res;
}

IDriver* HardwareManager::getActiveDriverInstance() {
    if (active_mode_.load(std::memory_order_relaxed) == HalMode::Realtime) {
        return real_driver_.get();
    }
    return sim_driver_.get();
}

AxisSet HardwareManager::applyCommandGovernor(const AxisSet& target) {
    auto now = std::chrono::steady_clock::now();
    
    if (is_first_cmd_) {
        last_sent_cmd_ = target;
        last_cmd_time_ = now;
        is_first_cmd_ = false;
        return target;
    }

    double dt_s = std::chrono::duration<double>(now - last_cmd_time_).count();
    if (dt_s <= 1e-6) {
        return last_sent_cmd_; 
    }

    AxisSet validated = last_sent_cmd_;
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
         auto& current_axis_validated = validated.GetAt(i).value().get();
         const auto& last_sent_axis = last_sent_cmd_.GetAt(i).value().get();
         const auto& target_axis = target.GetAt(i).value().get();
         
         double delta = target_axis.position.value() - last_sent_axis.position.value();
         double limit_vel = limits_.joint_velocity_limits_deg_s[i].value();

         if (std::abs(delta) / dt_s > limit_vel) {
             double max_delta = limit_vel * dt_s;
             current_axis_validated.position = Degrees(last_sent_axis.position.value() + ((delta > 0) ? max_delta : -max_delta));
         } else {
             current_axis_validated.position = target_axis.position;
         }
    }
    
    last_sent_cmd_ = validated;
    last_cmd_time_ = now;
    return validated;
}

} // namespace RDT
