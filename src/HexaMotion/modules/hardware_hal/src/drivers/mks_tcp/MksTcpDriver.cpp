// MksTcpDriver.cpp
#include "MksTcpDriver.h"
#include "LoggingMacros.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <string>
#include <string_view>

namespace RDT {

namespace {
constexpr int kHexaMotionClientId = 1;   ///< Mandatory client id; the server rejects anything else.
constexpr int kBroadcastAxisId = 0;      ///< axis_id used for global / telemetry requests.
constexpr int kSetZeroSettleFrames = 5;  ///< Telemetry frames to ignore after a SetZero (BUG-12).
/// Upper bound on the telemetry receive accumulator. A peer that never sends '\n' would otherwise
/// grow it without limit (same class as audit N1). The MKS backend is trusted and local, so on
/// overflow a warning + reset is sufficient here (no disconnect). Telemetry lines are small.
constexpr size_t kMaxRxAccumBytes = 1u * 1024u * 1024u;
constexpr uint8_t kOwnerHexaMotion = 1;  ///< control_owner_id value that means "HexaMotion owns motion".

// MKS AxisState (mirror of core::AxisState in the Motor Configurator).
enum class MksState : int {
    Unknown = 0,
    Disabled = 1,
    Ready = 2,
    OperationEnabled = 3,
    Fault = 4,
    Homing = 5,
    AutoHoming = 6,
    HomingOffset = 7
};

// HexaMotion axis index (0-based) -> MKS CAN id (1-based), per design decision.
[[nodiscard]] inline int axisIndexToCanId(int index) noexcept { return index + 1; }
} // namespace

MksTcpDriver::MksTcpDriver(const MksTcpConfig& config) : config_(config) {
    hal_state_.transport_ip = config_.ip;
    hal_state_.transport_port = config_.port;
    hal_state_.transport_connected = false;
    // offsets_ (AxisOffsets) self-initialises all axes to 0.0.
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        last_sent_target_deg_[i] = 0.0;
    }
}

MksTcpDriver::~MksTcpDriver() {
    stop();
}

Result<void, ErrorCode> MksTcpDriver::init() {
    if (is_running_.load()) {
        return Result<void, ErrorCode>::Success();
    }

    peer_ = std::make_unique<TcpPeer>(config_);
    if (!peer_) {
        RDT_LOG_CRITICAL(MODULE_NAME, "Failed to allocate TcpPeer.");
        return Result<void, ErrorCode>::Failure(ErrorCode::FilesystemError);
    }

    const int connect_result = peer_->connect();
    if (connect_result != 0) {
        RDT_LOG_ERROR(MODULE_NAME,
                      "Failed to connect to MKS Motor Configurator at {}:{}. Error code: {}",
                      config_.ip, config_.port, connect_result);
        peer_.reset();
        return Result<void, ErrorCode>::Failure(ErrorCode::SocketConnectFailed);
    }

    is_running_ = true;
    {
        std::lock_guard<std::mutex> lock(hal_state_mutex_);
        hal_state_.transport_connected = true;
    }
    worker_thread_ = std::jthread([this](std::stop_token st) { this->workerLoop(st); });

    // Claim control ownership and kick off the first telemetry exchange immediately so
    // the HardwareManager handshake (waits up to 2s for HalStatus::Ok) resolves quickly.
    sendTelemetryRequest();

    RDT_LOG_INFO(MODULE_NAME, "MKS TCP Driver initialized. Connected to {}:{}.",
                 config_.ip, config_.port);
    return Result<void, ErrorCode>::Success();
}

void MksTcpDriver::stop() {
    if (is_running_.exchange(false)) {
        RDT_LOG_INFO(MODULE_NAME, "Stopping MKS TCP Driver...");
        if (worker_thread_.joinable()) {
            worker_thread_.request_stop();
            worker_thread_.join();
        }
        // Guard peer teardown with socket_write_mutex_: the RT write path (sendRaw) may still call
        // peer_->send() under this same mutex from another thread, so resetting peer_ without it
        // would be a use-after-free of the socket. worker_thread_.join() above only stops the
        // receive/keep-alive worker, not the RT writer.
        {
            std::lock_guard<std::mutex> lock(socket_write_mutex_);
            if (peer_) {
                peer_->disconnect();
                peer_.reset();
            }
        }
        {
            std::lock_guard<std::mutex> lock(hal_state_mutex_);
            hal_state_.transport_connected = false;
        }
        RDT_LOG_INFO(MODULE_NAME, "MKS TCP Driver stopped.");
    }
}

Result<void, ErrorCode> MksTcpDriver::write(const AxisSet& cmd) {
    HardwareCommand command{};
    command.stream_target = cmd;
    return writeCommand(command);
}

Result<void, ErrorCode> MksTcpDriver::writeCommand(const HardwareCommand& command) {
    if (!is_running_.load()) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }

    // The MKS firmware does its own profiling, so we only push point-to-point segment
    // endpoints. The per-cycle 250Hz stream is intentionally ignored to keep the RT loop
    // off the TCP path (DoD #6).
    if (!command.has_segment_target) {
        return Result<void, ErrorCode>::Success();
    }

    std::array<double, ROBOT_AXES_COUNT> desired_phys{};
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        desired_phys[i] = offsets_.toPhysical(i, command.segment_target.target_angles[i].value());
    }
    // AXIS speed for this segment, scaled by the UI SPEED% (carried in segment_target.speed_ratio).
    // The Motor Configurator converts this axis deg/s into motor RPM (gear). Whether it is honored
    // is gated by the MC-side use_speed flag.
    const double speed_ratio = command.segment_target.speed_ratio;

    std::lock_guard<std::mutex> lock(send_mutex_);

    // Has the segment target actually changed since the last successful send?
    bool changed = !has_last_sent_;
    if (!changed) {
        for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            if (desired_phys[i] != last_sent_target_deg_[i]) { changed = true; break; }
        }
    }
    if (!changed) {
        return Result<void, ErrorCode>::Success();
    }

    // Throttle to send_rate_hz. If we are inside the window, stash the latest target as
    // "pending" so the worker thread flushes it once the window elapses. This guarantees
    // the final segment endpoint is delivered even if no further RT cycle calls us
    // (BUG-16) -- previously the last update inside the window could be lost.
    const auto now = std::chrono::steady_clock::now();
    const auto min_interval = std::chrono::milliseconds(
        config_.send_rate_hz > 0 ? 1000 / config_.send_rate_hz : 0);
    if (has_last_sent_ && (now - last_motion_send_) < min_interval) {
        pending_target_deg_ = desired_phys;
        pending_speed_ratio_ = speed_ratio;
        has_pending_target_ = true;
        return Result<void, ErrorCode>::Success();
    }

    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        if (!has_last_sent_ || desired_phys[i] != last_sent_target_deg_[i]) {
            auto res = sendMotionCommand(static_cast<int>(i), desired_phys[i], speed_ratio);
            if (res.isError()) {
                return res;
            }
        }
    }
    last_sent_target_deg_ = desired_phys;
    has_last_sent_ = true;
    has_pending_target_ = false;
    last_motion_send_ = now;
    return Result<void, ErrorCode>::Success();
}

void MksTcpDriver::flushPendingMotion() {
    std::lock_guard<std::mutex> lock(send_mutex_);
    if (!has_pending_target_) {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto min_interval = std::chrono::milliseconds(
        config_.send_rate_hz > 0 ? 1000 / config_.send_rate_hz : 0);
    if (has_last_sent_ && (now - last_motion_send_) < min_interval) {
        return; // still inside the throttle window; try again on the next worker tick.
    }

    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        if (!has_last_sent_ || pending_target_deg_[i] != last_sent_target_deg_[i]) {
            auto res = sendMotionCommand(static_cast<int>(i), pending_target_deg_[i], pending_speed_ratio_);
            if (res.isError()) {
                return; // socket error: keep pending and retry next tick.
            }
        }
    }
    last_sent_target_deg_ = pending_target_deg_;
    has_last_sent_ = true;
    has_pending_target_ = false;
    last_motion_send_ = now;
}

Result<HardwareFeedback, ErrorCode> MksTcpDriver::read() {
    if (!is_running_.load()) {
        return Result<HardwareFeedback, ErrorCode>::Failure(ErrorCode::NotConnected);
    }
    return Result<HardwareFeedback, ErrorCode>::Success(cached_feedback_.load(std::memory_order_acquire));
}

Result<void, ErrorCode> MksTcpDriver::masterAxisAt(AxisId axis, Degrees logical_position) {
    const int idx = AxisIdToInt(axis);
    if (idx < 0 || idx >= static_cast<int>(ROBOT_AXES_COUNT)) {
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidAxisId);
    }

    // SetZero makes the current physical position the hardware zero; we then store the
    // logical value as the offset so reads remain in the controller's logical frame.
    auto res = sendServiceCommand(idx, "SetZero");
    if (res.isError()) {
        return res;
    }
    // MKS mastering convention: SetZero makes the current physical position the hardware zero, so the
    // logical value IS the offset (differs from Sim/Udp offset = logical - physical, by design).
    offsets_.set(static_cast<std::size_t>(idx), logical_position.value());
    {
        // BUG-12: the next few telemetry frames still carry the pre-SetZero physical
        // position; combined with the freshly stored offset they would produce a
        // one-frame spike (old_phys + new_offset). Suppress them: processLine() reports
        // the mastered logical position until the hardware zero has propagated.
        std::lock_guard<std::mutex> lock(hal_state_mutex_);
        setzero_settle_frames_[static_cast<size_t>(idx)] = kSetZeroSettleFrames;
        // Audit F-01 follow-up: a fresh operator command supersedes the previous backend
        // rejection. Clear the sticky badge text now; if THIS command is also rejected the
        // response re-populates it one poll later (otherwise it stays cleared = recovered).
        hal_state_.last_ipc_error.clear();
    }
    RDT_LOG_INFO(MODULE_NAME, "Axis {} mastered to {} via SetZero.", idx + 1, logical_position.toString());
    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> MksTcpDriver::masterAllAxes() {
    if (!is_running_.load()) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }

    // One atomic SetZeroAll (the Motor Configurator fans it out to all axes), instead of six
    // separate SetZero commands. Format: {client_id, msg_type:"Service", cmd_type:"SetZeroAll", axis_ids}.
    nlohmann::json j;
    j["client_id"] = kHexaMotionClientId;
    j["msg_type"] = "Service";
    j["cmd_type"] = "SetZeroAll";
    nlohmann::json axis_ids = nlohmann::json::array();
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        axis_ids.push_back(axisIndexToCanId(static_cast<int>(i)));
    }
    j["axis_ids"] = axis_ids;

    std::string payload = j.dump();
    payload.push_back('\n');
    std::vector<char> data(payload.begin(), payload.end());
    if (sendRaw(data) < 0) {
        RDT_LOG_WARN(MODULE_NAME, "Failed to send Service 'SetZeroAll'.");
        return Result<void, ErrorCode>::Failure(ErrorCode::SocketSendFailed);
    }

    // Zero all logical offsets and suppress the post-SetZero telemetry spike (BUG-12) for every axis,
    // mirroring masterAxisAt() for logical 0.0.
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        offsets_.reset(i);
    }
    {
        std::lock_guard<std::mutex> lock(hal_state_mutex_);
        for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            setzero_settle_frames_[i] = kSetZeroSettleFrames;
        }
        // Audit F-01 follow-up: clear the sticky backend-rejection badge on a fresh attempt
        // (re-populated by the response only if this SetZeroAll is also rejected).
        hal_state_.last_ipc_error.clear();
    }
    RDT_LOG_INFO(MODULE_NAME, "All axes mastered via SetZeroAll.");
    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> MksTcpDriver::setBrakeState(bool engaged) {
    // MKS drives have no separate software brake line; this is a no-op.
    RDT_LOG_DEBUG(MODULE_NAME, "setBrakeState({}) ignored (no brake line on MKS).", engaged);
    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> MksTcpDriver::setHalConfig(const HalConfigState& hal_state) {
    if (!is_running_.load() || !peer_) {
        RDT_LOG_WARN(MODULE_NAME, "Rejected HAL config because MKS driver is not running.");
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }

    std::lock_guard<std::mutex> lock(hal_state_mutex_);
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        const bool want_enabled = hal_state.axes[i].motor_enabled;
        if (want_enabled != hal_state_.axes[i].motor_enabled) {
            auto res = sendServiceCommand(static_cast<int>(i), want_enabled ? "Enable" : "Disable");
            if (res.isError()) {
                return res;
            }
            // BUG-07: do NOT mirror the intent optimistically. The authoritative
            // motor_enabled flag is updated from telemetry in processLine() once the
            // drive actually reports OperationEnabled. If the command was lost or
            // rejected (ownership/E-STOP), the mirror stays false and the next
            // setHalConfig() call re-sends Enable instead of silently believing it
            // succeeded.
        }
        // Mirror soft/velocity limit config for getCurrentHalState() consistency.
        hal_state_.axes[i].soft_limits_enabled = hal_state.axes[i].soft_limits_enabled;
        hal_state_.axes[i].soft_limit_min = hal_state.axes[i].soft_limit_min;
        hal_state_.axes[i].soft_limit_max = hal_state.axes[i].soft_limit_max;
        hal_state_.axes[i].velocity_limit_enabled = hal_state.axes[i].velocity_limit_enabled;
        hal_state_.axes[i].velocity_limit = hal_state.axes[i].velocity_limit;
    }
    hal_state_.appliedRequestId = hal_state.appliedRequestId;
    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> MksTcpDriver::requestHoming(int axis_id) {
    if (!is_running_.load() || !peer_) return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);

    // Audit F-01 follow-up: a fresh homing request supersedes the previous backend rejection;
    // clear the sticky badge (re-populated by the response only if this request is rejected too).
    {
        std::lock_guard<std::mutex> lock(hal_state_mutex_);
        hal_state_.last_ipc_error.clear();
    }

    if (axis_id < 0) {
        // Homing is entirely the Motor Configurator's responsibility: HexaMotion only fires
        // the request and the configurator runs its own homing algorithm. We do NOT track
        // any homing state here.
        nlohmann::json j;
        j["client_id"] = kHexaMotionClientId;
        j["axis_id"] = 0; // The IPC controller bypasses the enqueue_service for HomeAllSequential
        j["msg_type"] = "Service";
        j["cmd_type"] = "HomeAllSequential";
        auto& arr = j["axis_ids"] = nlohmann::json::array();
        for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            arr.push_back(axisIndexToCanId(static_cast<int>(i)));
        }

        std::string payload = j.dump();
        payload.push_back('\n');
        std::vector<char> data(payload.begin(), payload.end());
        if (sendRaw(data) < 0) {
            return Result<void, ErrorCode>::Failure(ErrorCode::SocketSendFailed);
        }
        return Result<void, ErrorCode>::Success();
    }

    if (axis_id >= static_cast<int>(ROBOT_AXES_COUNT)) {
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidAxisId);
    }
    
    // Fire-and-forget: the configurator owns the homing algorithm and state.
    return sendServiceCommand(axis_id, "Home");
}

Result<void, ErrorCode> MksTcpDriver::emergencyStopAll() {
    if (!is_running_.load() || !peer_) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }

    // Ordinary service commands, fully subject to the configurator's ownership gate (project
    // decision: nothing bypasses it — with owner=UI these are rejected and the MC GUI E-Stop
    // is the physical stop). ClearMotionQueue first, then Disable, mirroring the order of the
    // configurator's own emergency_stop_all. Best-effort: keep going on errors so every axis
    // gets its chance; report the first failure.
    Result<void, ErrorCode> first_error = Result<void, ErrorCode>::Success();
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        auto clear_res = sendServiceCommand(static_cast<int>(i), "ClearMotionQueue");
        if (clear_res.isError() && first_error.isSuccess()) {
            first_error = clear_res;
        }
        auto disable_res = sendServiceCommand(static_cast<int>(i), "Disable");
        if (disable_res.isError() && first_error.isSuccess()) {
            first_error = disable_res;
        }
    }
    if (first_error.isError()) {
        RDT_LOG_WARN(MODULE_NAME, "EmergencyStopAll: at least one stop command failed to send.");
    } else {
        RDT_LOG_CRITICAL(MODULE_NAME, "EmergencyStopAll sent (ClearMotionQueue + Disable for all axes).");
    }
    return first_error;
}

Result<void, ErrorCode> MksTcpDriver::clearDriveErrors(int axis_id) {
    if (!is_running_.load() || !peer_) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }
    if (axis_id < 0) {
        for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            auto res = sendServiceCommand(static_cast<int>(i), "ClearErrors");
            if (res.isError()) {
                return res;
            }
        }
        return Result<void, ErrorCode>::Success();
    }
    if (axis_id >= static_cast<int>(ROBOT_AXES_COUNT)) {
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidAxisId);
    }
    return sendServiceCommand(axis_id, "ClearErrors");
}

HalConfigState MksTcpDriver::getCurrentHalState() const {
    std::lock_guard<std::mutex> lock(hal_state_mutex_);
    return hal_state_;
}

std::array<HalAxisConfigState, ROBOT_AXES_COUNT> MksTcpDriver::getAxisRuntimeState() const {
    // POD slice for the RT loop: no std::string members, no heap allocation.
    std::lock_guard<std::mutex> lock(hal_state_mutex_);
    return hal_state_.axes;
}

// --- Outgoing message builders -------------------------------------------------------

int MksTcpDriver::sendRaw(const std::vector<char>& data) {
    std::lock_guard<std::mutex> lock(socket_write_mutex_);
    if (!peer_) {
        return -1;
    }
    return peer_->send(data);
}

Result<void, ErrorCode> MksTcpDriver::setDigitalOutput(uint16_t port, bool state) {
    // Forward-compatible IPC command: {msg_type:"Service", cmd_type:"SetDigitalOutput", port, state}.
    // A Motor Configurator build without a DO channel answers with an explicit TelemetryResponse
    // error (surfaced as last_ipc_error in the HAL overlay) - never a silent no-op. The dry-run DO
    // path with loopback lives in the internal SimDriver.
    nlohmann::json j;
    j["client_id"] = kHexaMotionClientId;
    j["axis_id"] = 0;   // DO is not an axis command; field kept for the fixed request schema
    j["msg_type"] = "Service";
    j["cmd_type"] = "SetDigitalOutput";
    j["port"] = port;
    j["state"] = state;

    std::string payload = j.dump();
    payload.push_back('\n');
    std::vector<char> data(payload.begin(), payload.end());
    if (sendRaw(data) < 0) {
        RDT_LOG_WARN(MODULE_NAME, "Failed to send SetDigitalOutput DO[{}]={}.", port, state);
        return Result<void, ErrorCode>::Failure(ErrorCode::SocketSendFailed);
    }
    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> MksTcpDriver::sendServiceCommand(int axis_id, const char* cmd_type) {
    nlohmann::json j;
    j["client_id"] = kHexaMotionClientId;
    j["axis_id"] = axisIndexToCanId(axis_id);
    j["msg_type"] = "Service";
    j["cmd_type"] = cmd_type;

    std::string payload = j.dump();
    payload.push_back('\n');
    std::vector<char> data(payload.begin(), payload.end());
    if (sendRaw(data) < 0) {
        RDT_LOG_WARN(MODULE_NAME, "Failed to send Service '{}' for axis {}.", cmd_type, axis_id + 1);
        return Result<void, ErrorCode>::Failure(ErrorCode::SocketSendFailed);
    }
    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> MksTcpDriver::sendMotionCommand(int axis_index, double physical_target_deg, double speed_ratio) {
    nlohmann::json j;
    j["client_id"] = kHexaMotionClientId;
    j["axis_id"] = axisIndexToCanId(axis_index);
    j["msg_type"] = "Motion";
    j["target_pos"] = physical_target_deg;
    // AXIS speed (deg/s of the joint output) = configured 100% axis speed scaled by the UI SPEED%
    // (segment_target.speed_ratio). The Motor Configurator converts this to motor RPM (gear) and
    // clamps it; whether it honors this value or substitutes its own base is gated by the MC-side
    // use_speed flag. Floor the ratio at 5% so a near-zero override never freezes motion.
    j["target_vel"] = config_.default_vel_deg_s * std::clamp(speed_ratio, 0.05, 1.0);
    j["accel_pct"] = config_.default_accel_pct;
    j["is_relative"] = false;

    std::string payload = j.dump();
    payload.push_back('\n');
    std::vector<char> data(payload.begin(), payload.end());
    if (sendRaw(data) < 0) {
        RDT_LOG_WARN(MODULE_NAME, "Failed to send Motion for axis {}.", axis_index + 1);
        return Result<void, ErrorCode>::Failure(ErrorCode::SocketSendFailed);
    }
    return Result<void, ErrorCode>::Success();
}

void MksTcpDriver::sendTelemetryRequest() {
    nlohmann::json j;
    j["client_id"] = kHexaMotionClientId;
    j["axis_id"] = kBroadcastAxisId;
    j["msg_type"] = "Telemetry";

    std::string payload = j.dump();
    payload.push_back('\n');
    std::vector<char> data(payload.begin(), payload.end());

    (void)sendRaw(data);
}

// --- Incoming telemetry --------------------------------------------------------------

void MksTcpDriver::workerLoop(std::stop_token stoken) {
    RDT_LOG_INFO(MODULE_NAME, "MKS TCP worker thread started.");
    std::vector<char> buffer;
    std::string rx_accum;
    auto last_poll = std::chrono::steady_clock::now() - std::chrono::hours(1);
    const auto poll_interval = std::chrono::milliseconds(
        config_.telemetry_poll_ms > 0 ? config_.telemetry_poll_ms : 50);

    // Debounce sync-lost: a single empty receive (one missed poll, ~telemetry_poll_ms) is
    // normal jitter and must NOT flap driver_status Ok<->Warning_SyncLost. Only flag it after
    // telemetry has been silent for several poll cycles. This avoids a flickering HAL link
    // status (and the downstream Error/stopProgram cascade in REAL mode).
    auto last_rx = std::chrono::steady_clock::now();
    const auto sync_lost_threshold = (poll_interval * 4 > std::chrono::milliseconds(200))
        ? poll_interval * 4 : std::chrono::milliseconds(200);

    while (!stoken.stop_requested() && is_running_.load()) {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_poll >= poll_interval) {
            sendTelemetryRequest();
            last_poll = now;
        }

        // BUG-16: deliver any segment endpoint that writeCommand() throttled out.
        flushPendingMotion();

        const int bytes = peer_->receive(buffer);
        if (bytes > 0) {
            last_rx = std::chrono::steady_clock::now();
            rx_accum.append(buffer.data(), static_cast<size_t>(bytes));
            // Split on newline; the MKS IPC protocol is line-delimited JSON.
            size_t nl;
            while ((nl = rx_accum.find('\n')) != std::string::npos) {
                std::string_view line(rx_accum.data(), nl);
                if (!line.empty()) {
                    processLine(line);
                }
                rx_accum.erase(0, nl + 1);
            }
            // Bound the accumulator so an unterminated stream cannot grow it without limit (N1).
            if (rx_accum.size() > kMaxRxAccumBytes) {
                RDT_LOG_WARN(MODULE_NAME,
                             "Telemetry accumulator exceeded {} bytes without a complete line; resetting.",
                             kMaxRxAccumBytes);
                rx_accum.clear();
            }
        } else if (bytes == 0) {
            // Receive timeout. Only downgrade to Warning_SyncLost after sustained silence;
            // a single missed poll is normal jitter and must not flicker the link status.
            if (std::chrono::steady_clock::now() - last_rx > sync_lost_threshold) {
                HardwareFeedback current_fb = cached_feedback_.load(std::memory_order_acquire);
                if (current_fb.driver_status == HalStatus::Ok) {
                    current_fb.driver_status = HalStatus::Warning_SyncLost;
                    cached_feedback_.store(current_fb, std::memory_order_release);
                }
            }
        } else {
            HardwareFeedback current_fb = cached_feedback_.load(std::memory_order_acquire);
            current_fb.driver_status = HalStatus::Error_CommunicationLost;
            cached_feedback_.store(current_fb, std::memory_order_release);
            RDT_LOG_ERROR(MODULE_NAME, "MKS TCP receive error / peer closed. Attempting reconnect...");

            // BUG-05: do not give up. Retry the connection (with backoff) until it comes
            // back or a stop is requested. The previous code broke out of the loop, leaving
            // is_running_ == true with a dead link until the whole driver was restarted.
            if (!reconnect(stoken)) {
                break; // stop requested while retrying
            }
            rx_accum.clear();
            last_rx = std::chrono::steady_clock::now();
            last_poll = std::chrono::steady_clock::now() - poll_interval; // poll immediately
        }
    }
    RDT_LOG_INFO(MODULE_NAME, "MKS TCP worker thread finished.");
}

bool MksTcpDriver::reconnect(std::stop_token stoken) {
    auto backoff = std::chrono::milliseconds(200);
    const auto max_backoff = std::chrono::milliseconds(2000);

    while (!stoken.stop_requested() && is_running_.load()) {
        {
            // Hold the socket lock so no producer touches a half-open socket mid-reconnect.
            std::lock_guard<std::mutex> lock(socket_write_mutex_);
            if (peer_) {
                peer_->disconnect();
                if (peer_->connect() == 0) {
                    {
                        std::lock_guard<std::mutex> state_lock(hal_state_mutex_);
                        hal_state_.transport_connected = true;
                    }
                    RDT_LOG_INFO(MODULE_NAME, "MKS TCP reconnected to {}:{}.", config_.ip, config_.port);
                    return true;
                }
            }
        }
        {
            std::lock_guard<std::mutex> state_lock(hal_state_mutex_);
            hal_state_.transport_connected = false;
        }
        std::this_thread::sleep_for(backoff);
        backoff *= 2;
        if (backoff > max_backoff) {
            backoff = max_backoff;
        }
    }
    return false;
}

void MksTcpDriver::processLine(std::string_view line) {
    nlohmann::json root = nlohmann::json::parse(line, nullptr, /*allow_exceptions=*/false);
    if (root.is_discarded() || !root.is_object()) {
        RDT_LOG_WARN(MODULE_NAME, "Received malformed JSON line.");
        return;
    }

    const std::string msg_type = root.value("msg_type", std::string{});
    if (msg_type != "TelemetryResponse") {
        return; // Ignore unexpected message types.
    }

    const std::string error_text = root.value("error", std::string{});

    HardwareFeedback new_fb = cached_feedback_.load(std::memory_order_acquire);
    bool any_fault = false;
    bool any_moving = false;
    int axes_seen = 0;

    // BUG-02/BUG-18: surface who currently owns motion. When the owner is not
    // HexaMotion the configurator silently drops our commands; the UI needs to see this.
    const uint8_t owner_id = static_cast<uint8_t>(root.value("control_owner_id", static_cast<int>(kOwnerHexaMotion)));
    new_fb.control_owner_id = owner_id;
    // Mirror the owner into the HAL config state too, so it is serialized over RDT and the
    // HexaStudio HAL overlay can display the active control owner (Task 2 / audit F1).
    {
        std::lock_guard<std::mutex> lock(hal_state_mutex_);
        hal_state_.control_owner_id = owner_id;
        hal_state_.transport_connected = true;
        // Audit F-01/F-07: keep the backend's rejection text visible to the operator. Sticky:
        // the error rides only the response to the offending request (~one 50 ms poll), so it
        // is stored until the next error rather than cleared on the next clean response.
        if (!error_text.empty()) {
            hal_state_.last_ipc_error = error_text;
        }
    }

    // BUG-03: propagate the configurator's global E-STOP into the HAL safety state so the
    // RT core / HexaStudio react instead of streaming into a void.
    const bool estop_active = root.value("estop_active", false);
    new_fb.safety.is_estop_active = estop_active;

    // The MKS firmware runs its own trapezoidal profile to the commanded endpoint (P2P),
    // so HexaMotion does not interpolate per cycle. Marking the feedback as
    // InternalInterpolator tells MotionManager NOT to run its following-error check (the
    // commanded endpoint legitimately leads the lagging actual during a move; comparing the
    // two would falsely trip Error_FollowingError on far jogs).
    new_fb.execution_mode = HalCommandExecutionMode::InternalInterpolator;

    if (!error_text.empty()) {
        // We log the IPC application error, but we DO NOT return early: keep parsing the
        // 'axes' array so telemetry stays fresh. The rejection is reflected via
        // hal_state_.last_ipc_error / control_owner_id above.
        RDT_LOG_WARN(MODULE_NAME, "MKS IPC error: {} (control_owner_id={})", error_text, owner_id);
    }

    // Homing is entirely owned by the Motor Configurator. HexaMotion neither runs a homing
    // sequence nor drives a homing state machine. We only mirror the backend's
    // supervisor_status verbatim (read-only) so HexaStudio can show whether a HomeAll
    // sequence is running and at which stage. Per-axis homing phase is reflected separately
    // via the raw axis state (last_status) below.
    if (auto sup_it = root.find("supervisor_status"); sup_it != root.end() && sup_it->is_object()) {
        const auto& sup = *sup_it;
        std::lock_guard<std::mutex> lock(hal_state_mutex_);
        hal_state_.homing.sequence_active = sup.value("sequential_homing_active", false);
        hal_state_.homing.state = sup.value("state", std::string{});
        hal_state_.homing.current_index = sup.value("current_index", 0);
        hal_state_.homing.axis_count = sup.value("axis_count", 0);
        hal_state_.homing.current_axis_id = static_cast<uint16_t>(sup.value("current_axis_id", 0));
        hal_state_.homing.diagnostic = sup.value("diagnostic", std::string{});
    }

    auto axes_it = root.find("axes");
    if (axes_it != root.end() && axes_it->is_array()) {
        std::lock_guard<std::mutex> lock(hal_state_mutex_);
        for (const auto& ax : *axes_it) {
            const int can_id = ax.value("axis_id", 0);
            const int idx = can_id - 1;
            if (idx < 0 || idx >= static_cast<int>(ROBOT_AXES_COUNT)) {
                continue;
            }
            ++axes_seen;

            const double phys_pos = ax.value("pos", 0.0);
            const int state = ax.value("state", 0);
            const int protection = ax.value("protection", 0);
            const bool is_moving = ax.value("is_moving", false);

            const std::size_t axis_i = static_cast<std::size_t>(idx);
            // BUG-12: during the SetZero settle window report the mastered logical value (= the
            // offset) instead of stale_phys + offset.
            int& settle = setzero_settle_frames_[axis_i];
            const double logical_pos = (settle > 0) ? offsets_.get(axis_i)
                                                    : offsets_.toLogical(axis_i, phys_pos);
            if (settle > 0) --settle;
            new_fb.joints.GetAt(static_cast<size_t>(idx)).value().get().position = Degrees(logical_pos);

            if (is_moving) any_moving = true;

            const bool axis_fault = (state == static_cast<int>(MksState::Fault)) || (protection != 0);
            if (axis_fault) any_fault = true;

            auto& axis_state = hal_state_.axes[static_cast<size_t>(idx)];
            axis_state.last_status = state;
            axis_state.last_error_code = protection;
            axis_state.motor_enabled = (state == static_cast<int>(MksState::OperationEnabled));
            // Homing state intentionally not tracked here (owned by the configurator).
        }
    }

    new_fb.motion_active = any_moving;
    new_fb.target_reached = !any_moving;
    // BUG-11: an empty telemetry set means no axes are registered on the backend (CAN bus
    // down / no discovery). Reporting Ok here would let the HardwareManager handshake pass
    // with zero motors. Treat it as not-connected.
    if (axes_seen == 0) {
        new_fb.driver_status = HalStatus::NotConnected;
        new_fb.target_reached = false;
    } else {
        new_fb.driver_status = any_fault ? HalStatus::Error_DriveFault : HalStatus::Ok;
    }
    cached_feedback_.store(new_fb, std::memory_order_release);
}

} // namespace RDT
