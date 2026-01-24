// UdpDriver.h
#ifndef UDP_DRIVER_H
#define UDP_DRIVER_H

#pragma once

#include "../../interface/IDriver.h"
#include "UdpPeer.hpp"
#include <atomic>
#include <array>
#include <thread>
#include <memory>
#include <stop_token>

namespace RDT {

/**
 * @class UdpDriver
 * @brief Realtime driver for UDP-based hardware communication.
 *
 * This class spawns a background thread to continuously listen for UDP feedback packets.
 * The latest valid feedback is stored in an atomic cache, allowing the `read()` method
 * to be wait-free. `write()` sends commands via UDP.
 * It also implements the offset management required by the IDriver interface.
 */
class UdpDriver : public IDriver {
public:
    explicit UdpDriver(const UdpConfig& config);
    ~UdpDriver() override;

    // Non-copyable, non-movable due to thread and socket ownership
    UdpDriver(const UdpDriver&) = delete;
    UdpDriver& operator=(const UdpDriver&) = delete;
    UdpDriver(UdpDriver&&) = delete;
    UdpDriver& operator=(UdpDriver&&) = delete;

    Result<void, ErrorCode> init() override;
    void stop() override;
    Result<void, ErrorCode> write(const AxisSet& cmd) override;
    Result<HardwareFeedback, ErrorCode> read() override;
    Result<void, ErrorCode> resetAxisToZero(AxisId axis, double new_zero_deg) override;

private:
    void workerLoop(std::stop_token stoken);

    const UdpConfig config_;
    std::unique_ptr<UdpPeer> udp_peer_;
    std::jthread worker_thread_;
    std::atomic<bool> is_running_{false};

    // Atomically accessed cache for wait-free read()
    std::atomic<HardwareFeedback> cached_feedback_{};

    // Offsets: Logical = Physical + Offset
    std::array<std::atomic<double>, ROBOT_AXES_COUNT> offsets_;

    static inline const std::string MODULE_NAME = "UdpDriver";
};

} // namespace RDT

#endif // UDP_DRIVER_H
