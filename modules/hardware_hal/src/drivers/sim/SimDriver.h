// SimDriver.h
#ifndef SIM_DRIVER_H
#define SIM_DRIVER_H

#pragma once

#include "../../interface/IDriver.h"
#include <atomic>
#include <array>

namespace RDT {

/**
 * @class SimDriver
 * @brief Internal simulation backend.
 * Holds state in memory and supports Offset Management for testing calibration flows.
 */
class SimDriver : public IDriver {
public:
    explicit SimDriver(const AxisSet& initial_state);
    ~SimDriver() override = default;

    Result<void, ErrorCode> init() override;
    void stop() override;
    Result<void, ErrorCode> write(const AxisSet& cmd) override;
    Result<HardwareFeedback, ErrorCode> read() override;
    
    // Offset Management
    Result<void, ErrorCode> resetAxisToZero(AxisId axis, double new_zero_deg) override;

    // Sync Logic
    Result<void, ErrorCode> setState(const AxisSet& state) override;

private:
    // We store "Physical" state to properly simulate the offset logic.
    std::atomic<AxisSet> physical_state_;

    // To simulate 1-cycle latency (ideal drive), we store what we will report next.
    std::atomic<AxisSet> next_report_state_;
    
    // Offsets: Logical = Physical + Offset
    // Using atomic array for thread safety without mutexes in hot path
    std::atomic<double> offsets_[ROBOT_AXES_COUNT];
};

} // namespace RDT

#endif // SIM_DRIVER_H
