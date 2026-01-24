// HardwareManager.h
#ifndef HARDWARE_MANAGER_H
#define HARDWARE_MANAGER_H

#pragma once

#include "HalTypes.h"
#include "result.h"
#include "ErrorCode.h"
#include <memory>
#include <atomic>
#include <chrono>
#include <thread>

namespace RDT {

class IDriver;
struct InterfaceConfig;
struct RobotLimits;

/**
 * @class HardwareManager
 * @brief The public Facade and Guardian of the Hardware Abstraction Layer.
 */
class HardwareManager {
public:
    explicit HardwareManager(const InterfaceConfig& config, const RobotLimits& limits);
    ~HardwareManager();

    HardwareManager(const HardwareManager&) = delete;
    HardwareManager& operator=(const HardwareManager&) = delete;

    [[nodiscard]] Result<void, ErrorCode> init();
    void shutdown();

    [[nodiscard]] Result<void, ErrorCode> write(const AxisSet& cmd);
    [[nodiscard]] Result<HardwareFeedback, ErrorCode> read();

    [[nodiscard]] Result<void, ErrorCode> setMode(HalMode mode);
    [[nodiscard]] HalMode getMode() const;

    void syncSimulationToReal();
    Result<void, ErrorCode> zeroAxis(AxisId axis);

private:
    [[nodiscard]] IDriver* getActiveDriverInstance();
    [[nodiscard]] AxisSet applyCommandGovernor(const AxisSet& target);

    const InterfaceConfig& config_;
    const RobotLimits& limits_;

    std::unique_ptr<IDriver> sim_driver_;
    std::unique_ptr<IDriver> real_driver_;
    
    std::atomic<HalMode> active_mode_{HalMode::Simulation};

    AxisSet last_sent_cmd_{};
    std::chrono::steady_clock::time_point last_cmd_time_;
    bool is_first_cmd_{true};
};

} // namespace RDT

#endif // HARDWARE_MANAGER_H
