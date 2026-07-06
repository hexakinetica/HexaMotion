#include "SimDriver.h"
#include "LoggingMacros.h"

#include <string>

namespace RDT {

namespace {
constexpr const char* MODULE_NAME = "SimDriver";
} // namespace

SimDriver::SimDriver(const AxisSet& initial_state, const RobotLimits& limits) {
    physical_state_.store(initial_state);
    next_report_state_.store(initial_state);
    // offsets_ (AxisOffsets) self-initialises all axes to 0.0.
    // Simulated motors are ENABLED out of the box (boss directive 2026-07-06): the sim stub must
    // be usable for debugging immediately, with no physical drive to protect. Brakes released to
    // stay consistent with setHalConfig()'s any-motor-enabled rule.
    m_brakesEngaged.store(false);

    // HAL limits mirror the ROBOT limits (REQ-SIMMC-09): the command governor clamps the
    // REAL-view stream to these values, so they must agree with what the planner is allowed —
    // a lower arbitrary default (the old 100 deg/s hardcode) made the actual lag the command
    // and tripped Error_FollowingError on fast moves. The HAL panel can still lower them live.
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        hal_state_.axes[i].motor_enabled = true;
        hal_state_.axes[i].soft_limits_enabled = true;
        hal_state_.axes[i].soft_limit_min = limits.joint_position_limits_deg[i].first;
        hal_state_.axes[i].soft_limit_max = limits.joint_position_limits_deg[i].second;
        hal_state_.axes[i].velocity_limit_enabled = true;
        hal_state_.axes[i].velocity_limit = limits.joint_velocity_limits_deg_s[i];
        hal_state_.axes[i].last_status = static_cast<int>(HalStatus::Ok);
        hal_state_.axes[i].last_error_code = 0;
    }
}

Result<void, ErrorCode> SimDriver::init() {
    return Result<void, ErrorCode>::Success();
}

void SimDriver::stop() {
    // Nothing to stop in sim
}

Result<void, ErrorCode> SimDriver::write(const AxisSet& cmd) {
    AxisSet physical_cmd = cmd;

    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        auto current_val = cmd.GetAt(i).value().get().position.value();
        physical_cmd.GetAt(i).value().get().position = Degrees(offsets_.toPhysical(i, current_val));
    }

    next_report_state_.store(physical_cmd);
    
    return Result<void, ErrorCode>::Success();
}

Result<HardwareFeedback, ErrorCode> SimDriver::read() {
    AxisSet raw = physical_state_.load();
    physical_state_.store(next_report_state_.load());

    HardwareFeedback fb;
    
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        auto raw_val = raw.GetAt(i).value().get().position.value();
        fb.joints.GetAt(i).value().get().position = Degrees(offsets_.toLogical(i, raw_val));
    }

    fb.driver_status = HalStatus::Ok;
    fb.safety.is_power_on = true;
    // SIM LOOPBACK: program-driven digital outputs are mirrored onto the inputs, so SET DO +
    // WAIT DI/IF programs are dry-runnable in pure simulation (documented sim behaviour).
    fb.digital_inputs = digital_outputs_.load(std::memory_order_relaxed);

    return Result<HardwareFeedback, ErrorCode>::Success(fb);
}

Result<void, ErrorCode> SimDriver::masterAxisAt(AxisId axis, Degrees logical_position) {
    int idx = static_cast<int>(axis);
    if (idx < 0 || idx >= ROBOT_AXES_COUNT) {
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidAxisId);
    }

    AxisSet current_phys = physical_state_.load();
    double phys_val = current_phys.GetAt(idx).value().get().position.value();

    // Sim/Udp mastering convention: offset = logical - physical (kept in the driver by design).
    double new_offset = logical_position.value() - phys_val;
    offsets_.set(static_cast<std::size_t>(idx), new_offset);

    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> SimDriver::setState(const AxisSet& state) {
    AxisSet new_phys = state;
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        auto logic_val = state.GetAt(i).value().get().position.value();
        new_phys.GetAt(i).value().get().position = Degrees(offsets_.toPhysical(i, logic_val));
    }
    
    physical_state_.store(new_phys);
    next_report_state_.store(new_phys);
    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> SimDriver::setBrakeState(bool engaged) {
    m_brakesEngaged.store(engaged);
    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> SimDriver::setHalConfig(const HalConfigState& hal_state) {
    std::lock_guard<std::mutex> lock(hal_state_mutex_);
    hal_state_ = hal_state;

    bool any_motor_enabled = false;
    bool all_motors_disabled = true;
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        any_motor_enabled = any_motor_enabled || hal_state_.axes[i].motor_enabled;
        all_motors_disabled = all_motors_disabled && !hal_state_.axes[i].motor_enabled;
        hal_state_.axes[i].last_status = static_cast<int>(HalStatus::Ok);
        hal_state_.axes[i].last_error_code = 0;
    }

    if (all_motors_disabled) {
        m_brakesEngaged.store(true);
    } else if (any_motor_enabled) {
        m_brakesEngaged.store(false);
    }

    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> SimDriver::requestHoming(int axis_id) {
    if (axis_id < -1 || axis_id >= static_cast<int>(ROBOT_AXES_COUNT)) {
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidAxisId);
    }

    // Simulated homing (boss directive 2026-07-07: the sim stub must behave adequately, not
    // refuse). A real drive seeks its physical home switch; the simulator's equivalent is the
    // axis arriving at logical zero instantly. -1 homes all axes (mirrors the wire contract).
    // The caller (RobotController) resyncs the RT hold target afterwards, exactly like the
    // native-HAL homing resync path.
    AxisSet target = physical_state_.load();
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        auto raw_val = target.GetAt(i).value().get().position.value();
        target.GetAt(i).value().get().position = Degrees(offsets_.toLogical(i, raw_val));
    }
    if (axis_id == -1) {
        for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            target.GetAt(i).value().get().position = Degrees(0.0);
        }
    } else {
        target.GetAt(static_cast<size_t>(axis_id)).value().get().position = Degrees(0.0);
    }
    RDT_LOG_INFO(MODULE_NAME, "Sim homing: axis {} -> logical 0.",
                 axis_id == -1 ? std::string("ALL") : std::to_string(axis_id + 1));
    return setState(target);
}

HalConfigState SimDriver::getCurrentHalState() const {
    std::lock_guard<std::mutex> lock(hal_state_mutex_);
    return hal_state_;
}

std::array<HalAxisConfigState, ROBOT_AXES_COUNT> SimDriver::getAxisRuntimeState() const {
    // POD slice for the RT loop: no std::string members, no heap allocation.
    std::lock_guard<std::mutex> lock(hal_state_mutex_);
    return hal_state_.axes;
}


Result<void, ErrorCode> SimDriver::setDigitalOutput(uint16_t port, bool state) {
    // 1-based port; the 32-bit output word bounds the valid range explicitly.
    if (port < 1 || port > 32) {
        RDT_LOG_ERROR(MODULE_NAME, "SetDigitalOutput refused: port {} is outside 1..32.", port);
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidArgument);
    }
    const uint32_t bit = 1u << (port - 1);
    uint32_t bits = digital_outputs_.load(std::memory_order_relaxed);
    if (state) bits |= bit; else bits &= ~bit;
    digital_outputs_.store(bits, std::memory_order_relaxed);
    RDT_LOG_INFO(MODULE_NAME, "Sim DO[{}] = {} (loopback to DI).", port, state ? "HIGH" : "LOW");
    return Result<void, ErrorCode>::Success();
}
} // namespace RDT
