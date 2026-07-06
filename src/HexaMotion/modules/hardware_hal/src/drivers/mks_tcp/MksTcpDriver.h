// MksTcpDriver.h
#pragma once

#include "../../interface/IDriver.h"
#include "../AxisOffsets.h"
#include "TcpPeer.hpp"
#include <atomic>
#include <array>
#include <chrono>
#include <mutex>
#include <thread>
#include <memory>
#include <stop_token>

namespace RDT {

/**
 * @class MksTcpDriver
 * @brief Realtime driver that talks to the MKS CAN Motor Configurator over JSON/TCP.
 *
 * HexaMotion is the single, exclusive TCP client (client_id == 1) of the Motor
 * Configurator IPC server (default port 30110). The protocol is newline-delimited
 * JSON. This driver:
 *  - streams motion as point-to-point **segment endpoints** (the MKS firmware runs its
 *    own trapezoidal profile), throttled to @ref MksTcpConfig::send_rate_hz;
 *  - runs a background thread that periodically polls Telemetry and parses the latest
 *    `TelemetryResponse` into an atomic @ref HardwareFeedback cache for wait-free read();
 *  - maps HAL service intents (Enable/Disable/Home/SetZero/ClearErrors) onto IPC commands.
 *
 * Mirrors the structure of @ref UdpDriver, including the Logical = Physical + Offset model.
 */
class MksTcpDriver : public IDriver {
public:
    explicit MksTcpDriver(const MksTcpConfig& config);
    ~MksTcpDriver() override;

    // Non-copyable, non-movable due to thread and socket ownership.
    MksTcpDriver(const MksTcpDriver&) = delete;
    MksTcpDriver& operator=(const MksTcpDriver&) = delete;
    MksTcpDriver(MksTcpDriver&&) = delete;
    MksTcpDriver& operator=(MksTcpDriver&&) = delete;

    Result<void, ErrorCode> init() override;
    void stop() override;
    Result<void, ErrorCode> write(const AxisSet& cmd) override;
    Result<void, ErrorCode> writeCommand(const HardwareCommand& command) override;
    Result<HardwareFeedback, ErrorCode> read() override;
    Result<void, ErrorCode> masterAxisAt(AxisId axis, Degrees logical_position) override;
    Result<void, ErrorCode> masterAllAxes() override;   ///< Single atomic SetZeroAll for all axes.
    [[nodiscard]] Result<void, ErrorCode> setBrakeState(bool engaged) override;
    Result<void, ErrorCode> setHalConfig(const HalConfigState& hal_state) override;
    Result<void, ErrorCode> requestHoming(int axis_id) override;
    Result<void, ErrorCode> setDigitalOutput(uint16_t port, bool state) override;
    Result<void, ErrorCode> clearDriveErrors(int axis_id) override;
    /// Per-axis ClearMotionQueue + Disable for every axis (best-effort) — the MKS firmware
    /// profiles to the commanded endpoint on its own, so an E-Stop must actively stop it (F-13).
    [[nodiscard]] Result<void, ErrorCode> emergencyStopAll() override;
    [[nodiscard]] HalConfigState getCurrentHalState() const override;
    [[nodiscard]] std::array<HalAxisConfigState, ROBOT_AXES_COUNT> getAxisRuntimeState() const override;

private:
    void workerLoop(std::stop_token stoken);
    void processLine(std::string_view line);

    // BUG-16: flush a segment endpoint that was throttled out of writeCommand() once the
    // send-rate window elapses, so the axis always reaches the final commanded point.
    void flushPendingMotion();

    // BUG-05: re-establish the TCP link after a drop. Returns false only if a stop was
    // requested while retrying.
    [[nodiscard]] bool reconnect(std::stop_token stoken);

    // Helpers that build & send one newline-delimited JSON message (thread-safe).
    [[nodiscard]] Result<void, ErrorCode> sendServiceCommand(int axis_id, const char* cmd_type);
    [[nodiscard]] Result<void, ErrorCode> sendMotionCommand(int axis_index, double physical_target_deg, double speed_ratio);
    void sendTelemetryRequest();

    // Single choke-point for every socket write. Serializes the actual send so that
    // concurrent producers (RT thread, telemetry poller, service calls) can never
    // interleave the bytes of two newline-delimited JSON lines (BUG-04).
    [[nodiscard]] int sendRaw(const std::vector<char>& data);

    const MksTcpConfig config_;
    std::unique_ptr<TcpPeer> peer_;
    std::jthread worker_thread_;
    std::atomic<bool> is_running_{false};

    // Lock-free telemetry cache for read(): HardwareFeedback is trivially copyable, so the
    // worker thread publishes a whole snapshot atomically and read() returns a copy without
    // taking a lock. (Note: on this large a payload the standard library may implement the
    // atomic with an internal lock; it is still race-free, just not literally wait-free.)
    std::atomic<HardwareFeedback> cached_feedback_{};

    // Offsets: Logical = Physical + Offset (shared helper, audit B3).
    AxisOffsets offsets_;

    // Serializes the actual socket write (innermost lock; see sendRaw).
    std::mutex socket_write_mutex_;

    // Guards the motion throttle / change-detection bookkeeping below.
    std::mutex send_mutex_;

    // Motion throttling / change detection (guarded by send_mutex_).
    std::chrono::steady_clock::time_point last_motion_send_{};
    std::array<double, ROBOT_AXES_COUNT> last_sent_target_deg_{};
    bool has_last_sent_{false};

    // Pending segment endpoint that was throttled out (BUG-16): if the target stops
    // changing inside the send-rate window, the worker thread flushes it so the axis
    // always reaches the final commanded point.
    std::array<double, ROBOT_AXES_COUNT> pending_target_deg_{};
    double pending_speed_ratio_{1.0}; // axis speed scale carried with the pending segment endpoint
    bool has_pending_target_{false};

    // Per-axis SetZero settle guard (BUG-12): ignore stale physical positions for a few
    // telemetry frames after a SetZero so the offset does not produce a position spike.
    std::array<int, ROBOT_AXES_COUNT> setzero_settle_frames_{};

    // HAL runtime state mirror (homing state machine, enable mirror).
    mutable std::mutex hal_state_mutex_;
    HalConfigState hal_state_{};

    static inline const std::string MODULE_NAME = "MksTcpDriver";
};

} // namespace RDT
