#include "SimDriver.h"

namespace RDT {

SimDriver::SimDriver(const AxisSet& initial_state) {
    physical_state_.store(initial_state);
    next_report_state_.store(initial_state);
    
    // Initialize offsets to 0.0
    for (int i = 0; i < ROBOT_AXES_COUNT; ++i) {
        offsets_[i].store(0.0);
    }
}

Result<void, ErrorCode> SimDriver::init() {
    return Result<void, ErrorCode>::Success();
}

void SimDriver::stop() {
    // Nothing to stop in sim
}

Result<void, ErrorCode> SimDriver::write(const AxisSet& cmd) {
    // In Simulation, 'write' sets the Physical state instantly, 
    // but we must respect the Offset: Physical = Logical - Offset.
    // This allows testing: "If I zero Axis 1, does the Logical position become 0 while Physical stays same?"
    
    AxisSet physical_cmd = cmd;
    
    // Apply Inverse Offset logic
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        double off = offsets_[i].load(std::memory_order_relaxed);
        // We need mutable access to the axis in AxisSet. 
        // Assuming AxisSet allows modification via GetAt reference or we reconstruct.
        // For simplicity/performance in this block:
        auto current_val = cmd.GetAt(i).value().get().position.value();
        physical_cmd.GetAt(i).value().get().position = Degrees(current_val - off);
    }

    // Simulate 1-cycle delay:
    // The state we just wrote will be available for reading in the NEXT cycle.
    // The state available for THIS cycle is what was written previously (stored in physical_state_).
    next_report_state_.store(physical_cmd);
    
    return Result<void, ErrorCode>::Success();
}

Result<HardwareFeedback, ErrorCode> SimDriver::read() {
    // Simulate 1-cycle delay:
    // We read the state that was "active" at the start of the cycle (physical_state_),
    // NOT the state that was just written in this cycle (next_report_state_).
    // After reading, we update physical_state_ to the next state for the next cycle.
    
    AxisSet raw = physical_state_.load();
    physical_state_.store(next_report_state_.load()); // Advance simulation time

    HardwareFeedback fb;
    
    // Apply Forward Offset logic: Logical = Physical + Offset
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        double off = offsets_[i].load(std::memory_order_relaxed);
        auto raw_val = raw.GetAt(i).value().get().position.value();
        fb.joints.GetAt(i).value().get().position = Degrees(raw_val + off);
    }

    fb.driver_status = HalStatus::Ok;
    fb.safety.is_power_on = true; 
    
    return Result<HardwareFeedback, ErrorCode>::Success(fb);
}

Result<void, ErrorCode> SimDriver::resetAxisToZero(AxisId axis, double new_zero_deg) {
    int idx = static_cast<int>(axis);
    if (idx < 0 || idx >= ROBOT_AXES_COUNT) {
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidAxisId);
    }

    // Logic: NewOffset = TargetLogical - CurrentPhysical
    AxisSet current_phys = physical_state_.load();
    double phys_val = current_phys.GetAt(idx).value().get().position.value();
    
    double new_offset = new_zero_deg - phys_val;
    offsets_[idx].store(new_offset);

    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> SimDriver::setState(const AxisSet& state) {
    // For syncing Sim to Real, we usually assume Offsets match or we reset them?
    // Usually, we just hard-set the physical state to match the requested logical state
    // assuming offsets are zero, OR we adjust physical to produce this logical.
    // Let's adjust Physical to produce the requested Logical with current offsets.
    
    AxisSet new_phys = state;
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        double off = offsets_[i].load(std::memory_order_relaxed);
        auto logic_val = state.GetAt(i).value().get().position.value();
        new_phys.GetAt(i).value().get().position = Degrees(logic_val - off);
    }
    
    physical_state_.store(new_phys);
    next_report_state_.store(new_phys); // Also update next state to avoid jumps
    return Result<void, ErrorCode>::Success();
}

} // namespace RDT
