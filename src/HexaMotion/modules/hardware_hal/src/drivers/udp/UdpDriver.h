// UdpDriver.h
#pragma once

#include "../../interface/IDriver.h"
#include "../AxisOffsets.h"
#include "UdpPeer.hpp"
#include <atomic>
#include <array>
#include <mutex>
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
    Result<void, ErrorCode> writeCommand(const HardwareCommand& command) override;
    Result<HardwareFeedback, ErrorCode> read() override;
    Result<void, ErrorCode> masterAxisAt(AxisId axis, Degrees logical_position) override;
    [[nodiscard]] Result<void, ErrorCode> setBrakeState(bool engaged) override;
    Result<void, ErrorCode> setHalConfig(const HalConfigState& hal_state) override;
    Result<void, ErrorCode> requestHoming(int axis_id) override;
    [[nodiscard]] HalConfigState getCurrentHalState() const override;
    [[nodiscard]] std::array<HalAxisConfigState, ROBOT_AXES_COUNT> getAxisRuntimeState() const override;

private:
    void workerLoop(std::stop_token stoken);

    const UdpConfig config_;
    std::unique_ptr<UdpPeer> udp_peer_;
    std::jthread worker_thread_;
    std::atomic<bool> is_running_{false};

    // Atomically accessed cache for wait-free read()
    std::atomic<HardwareFeedback> cached_feedback_{};

    // Offsets: Logical = Physical + Offset (shared helper, audit B3).
    AxisOffsets offsets_;
    mutable std::mutex hal_state_mutex_;
    HalConfigState hal_state_{};

    static inline const std::string MODULE_NAME = "UdpDriver";
};

} // namespace RDT
