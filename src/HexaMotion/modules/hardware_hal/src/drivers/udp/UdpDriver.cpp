#include "UdpDriver.h"
#include "LoggingMacros.h"
#include <sstream>
#include <vector>
#include <charconv>
#include <chrono>
#include <iomanip>

namespace RDT {

UdpDriver::UdpDriver(const UdpConfig& config) : config_(config) {
    // offsets_ (AxisOffsets) self-initialises all axes to 0.0.
}

UdpDriver::~UdpDriver() {
    stop();
}

Result<void, ErrorCode> UdpDriver::init() {
    if (is_running_.load()) {
        return Result<void, ErrorCode>::Success();
    }

    // No try/catch. Explicitly create and check the pointer.
    udp_peer_ = std::make_unique<UdpPeer>(config_);
    if (!udp_peer_) {
        // This would only happen in an extreme out-of-memory scenario.
        RDT_LOG_CRITICAL(MODULE_NAME, "Failed to allocate memory for UdpPeer.");
        return Result<void, ErrorCode>::Failure(ErrorCode::FilesystemError); // Using FilesystemError as a proxy for memory allocation failure.
    }
    
    int connect_result = udp_peer_->connect();
    if (connect_result != 0) {
        RDT_LOG_ERROR(MODULE_NAME, "Failed to bind/connect UDP peer. Error code: {}", connect_result);
        udp_peer_.reset();
        return Result<void, ErrorCode>::Failure(ErrorCode::SocketBindFailed);
    }

    is_running_ = true;
    worker_thread_ = std::jthread([this](std::stop_token st) { this->workerLoop(st); });
    
    // [FIX] Deadlock prevention: Send a PING packet to wake up the UDP HAL peer / robot.
    // The UDP HAL peer waits for a packet before replying. HardwareManager waits for a reply before proceeding.
    // Without this ping, they both wait forever (timeout 2s).
    if (is_running_.load()) {
        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        std::string ping_msg = "[PING] TS=" + std::to_string(ms);
        std::vector<char> ping_data(ping_msg.begin(), ping_msg.end());
        udp_peer_->send(ping_data);
    }

    RDT_LOG_INFO(MODULE_NAME, "UDP Driver initialized. Listening on port {}. Sending to {}:{}.",
        config_.local_port, config_.remote_ip, config_.remote_port);
    return Result<void, ErrorCode>::Success();
}

void UdpDriver::stop() {
    if (is_running_.exchange(false)) {
        RDT_LOG_INFO(MODULE_NAME, "Stopping UDP Driver...");
        if (worker_thread_.joinable()) {
            worker_thread_.request_stop();
            worker_thread_.join();
        }
        if (udp_peer_) {
            udp_peer_->disconnect();
            udp_peer_.reset();
        }
        RDT_LOG_INFO(MODULE_NAME, "UDP Driver stopped.");
    }
}

Result<void, ErrorCode> UdpDriver::write(const AxisSet& cmd) {
    HardwareCommand command{};
    command.stream_target = cmd;
    return writeCommand(command);
}

Result<void, ErrorCode> UdpDriver::writeCommand(const HardwareCommand& command) {
    if (!is_running_.load()) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }
    
    AxisSet phys_cmd = command.stream_target;
    std::array<double, ROBOT_AXES_COUNT> phys_target_deg{};
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        auto& axis = phys_cmd.GetAt(i).value().get();
        axis.position = Degrees(offsets_.toPhysical(i, axis.position.value()));

        const double logical_target_deg = command.has_segment_target
            ? command.segment_target.target_angles[i].value()
            : command.stream_target.GetAt(i).value().get().position.value();
        phys_target_deg[i] = offsets_.toPhysical(i, logical_target_deg);
    }

    std::stringstream oss;
    oss.precision(4);
    oss << std::fixed;
    oss << "[CMD]";
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        oss << " A" << (i + 1) << "=" << phys_cmd.GetAt(i).value().get().position.value();
    }
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        oss << " T" << (i + 1) << "=" << phys_target_deg[i];
    }
    
    // [FEATURE] Add Timestamp (ms) for synchronization
    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    oss << " TS=" << ms;
    
    std::string packet_str = oss.str();
    std::vector<char> data(packet_str.begin(), packet_str.end());
    
    if (udp_peer_->send(data) < 0) {
        RDT_LOG_WARN(MODULE_NAME, "UDP send failed.");
        return Result<void, ErrorCode>::Failure(ErrorCode::SocketSendFailed);
    }
    return Result<void, ErrorCode>::Success();
}

Result<HardwareFeedback, ErrorCode> UdpDriver::read() {
    if (!is_running_.load()) {
        return Result<HardwareFeedback, ErrorCode>::Failure(ErrorCode::NotConnected);
    }
    return Result<HardwareFeedback, ErrorCode>::Success(cached_feedback_.load(std::memory_order_acquire));
}

Result<void, ErrorCode> UdpDriver::masterAxisAt(AxisId axis, Degrees logical_position) {
    int idx = AxisIdToInt(axis);
    if (idx < 0 || idx >= ROBOT_AXES_COUNT) {
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidAxisId);
    }

    HardwareFeedback current_fb = cached_feedback_.load(std::memory_order_acquire);
    double current_logical_deg = current_fb.joints.GetAt(idx).value().get().position.value();
    double current_offset_deg = offsets_.get(static_cast<std::size_t>(idx));

    // Sim/Udp mastering convention: offset = logical - physical (kept in the driver by design).
    double current_physical_deg = current_logical_deg - current_offset_deg;
    double new_offset_deg = logical_position.value() - current_physical_deg;

    offsets_.set(static_cast<std::size_t>(idx), new_offset_deg);
    
    RDT_LOG_INFO(MODULE_NAME, "Axis {} mastered to {}. Physical: {:.3f} deg, New Offset: {:.3f} deg.", idx + 1, logical_position.toString(), current_physical_deg, new_offset_deg);
    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> UdpDriver::setBrakeState(bool engaged) {
    if (!is_running_.load() || !udp_peer_) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }

    const std::string packet = engaged ? "[BRK] STATE=1" : "[BRK] STATE=0";
    std::vector<char> data(packet.begin(), packet.end());
    if (udp_peer_->send(data) < 0) {
        RDT_LOG_WARN(MODULE_NAME, "Failed to send brake command over UDP.");
        return Result<void, ErrorCode>::Failure(ErrorCode::SocketSendFailed);
    }

    RDT_LOG_INFO(MODULE_NAME, "Brake state command sent: {}", engaged ? "ENGAGED" : "RELEASED");
    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> UdpDriver::setHalConfig(const HalConfigState& hal_state) {
    if (!is_running_.load() || !udp_peer_) {
        RDT_LOG_WARN(MODULE_NAME, "Rejected HAL packet send because UDP driver is not running.");
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }

    std::stringstream oss;
    oss << std::fixed << std::setprecision(4);
    oss << "[HAL]";
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        const auto axis_num = i + 1;
        const auto& axis = hal_state.axes[i];
        oss << " M" << axis_num << "=" << (axis.motor_enabled ? "1" : "0");
        oss << " B" << axis_num << "=" << (axis.motor_enabled ? "0" : "1");
        oss << " SL" << axis_num << "=" << (axis.soft_limits_enabled ? "1" : "0");
        oss << " MIN" << axis_num << "=" << axis.soft_limit_min.value();
        oss << " MAX" << axis_num << "=" << axis.soft_limit_max.value();
        oss << " VL" << axis_num << "=" << axis.velocity_limit.value();
    }

    std::string packet = oss.str();
    RDT_LOG_INFO(MODULE_NAME,
                 "Sending HAL packet to UDP HAL peer. request_id={}, payload={}",
                 hal_state.appliedRequestId,
                 packet);
    std::vector<char> data(packet.begin(), packet.end());
    if (udp_peer_->send(data) < 0) {
        RDT_LOG_WARN(MODULE_NAME, "Failed to send HAL packet over UDP.");
        return Result<void, ErrorCode>::Failure(ErrorCode::SocketSendFailed);
    }
    {
        std::lock_guard<std::mutex> lock(hal_state_mutex_);
        hal_state_ = hal_state;
    }
    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> UdpDriver::requestHoming(int axis_id) {
    if (!is_running_.load() || !udp_peer_) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }

    std::string packet = axis_id < 0 ? "[HOME] AXIS=ALL" : ("[HOME] AXIS=" + std::to_string(axis_id + 1));
    std::vector<char> data(packet.begin(), packet.end());
    if (udp_peer_->send(data) < 0) {
        return Result<void, ErrorCode>::Failure(ErrorCode::SocketSendFailed);
    }
    // Fire-and-forget: homing progress is owned by the backend and reflected through the
    // raw axis state (last_status), not tracked here.
    return Result<void, ErrorCode>::Success();
}

HalConfigState UdpDriver::getCurrentHalState() const {
    std::lock_guard<std::mutex> lock(hal_state_mutex_);
    return hal_state_;
}

std::array<HalAxisConfigState, ROBOT_AXES_COUNT> UdpDriver::getAxisRuntimeState() const {
    // POD slice for the RT loop: no std::string members, no heap allocation.
    std::lock_guard<std::mutex> lock(hal_state_mutex_);
    return hal_state_.axes;
}

void UdpDriver::workerLoop(std::stop_token stoken) {
    RDT_LOG_INFO(MODULE_NAME, "UDP worker thread started.");
    std::vector<char> buffer;
    
    while (!stoken.stop_requested() && is_running_.load()) {
        int bytes = udp_peer_->receive(buffer);
        
        if (bytes > 0) {
            std::string_view msg(buffer.data(), bytes);
            
            if (!msg.starts_with("[FB]")) {
                RDT_LOG_WARN(MODULE_NAME, "Received malformed UDP packet: {}", msg);
                continue;
            }

            HardwareFeedback new_fb;
            AxisSet raw_phys_joints;
            
            size_t start_pos = 4; // Skip "[FB]"
            
            while (start_pos < msg.length()) {
                if (msg[start_pos] == ' ' || msg[start_pos] == ',') {
                    start_pos++;
                    continue;
                }
                size_t token_end = msg.find_first_of(" ,", start_pos);
                if (token_end == std::string_view::npos) token_end = msg.length();
                std::string_view token = msg.substr(start_pos, token_end - start_pos);
                start_pos = token_end + 1;
                
                size_t eq_pos = token.find('=');
                if (eq_pos != std::string_view::npos) {
                    std::string_view key = token.substr(0, eq_pos);
                    std::string_view val_str = token.substr(eq_pos + 1);
                    
                    // Guard key[1] against a lone "A" token (string_view has no bounds check, so
                    // key[1] on a length-1 view is out-of-bounds/UB on malformed peer input).
                    if (key.starts_with("A") && key.size() >= 2) {
                        int axis_idx = key[1] - '1';
                        if (axis_idx >= 0 && axis_idx < ROBOT_AXES_COUNT) {
                            double val_deg = 0.0;
                            std::from_chars(val_str.data(), val_str.data() + val_str.size(), val_deg);
                            raw_phys_joints.GetAt(axis_idx).value().get().position = Degrees(val_deg);
                        }
                    } else if (key.starts_with("DI")) {
                        int di_idx = 0;
                        std::from_chars(key.data() + 2, key.data() + key.size(), di_idx);
                        // Guard the shift: di_idx-1 must be a valid bit index. A DI0 or a parse
                        // failure (di_idx == 0) would otherwise shift by -1, and di_idx > 32 would
                        // shift past the field width — both undefined behaviour on malformed input.
                        constexpr int kMaxDigitalInputs = 32;
                        if (val_str == "1" && di_idx >= 1 && di_idx <= kMaxDigitalInputs) {
                            new_fb.digital_inputs |= (1u << (di_idx - 1));
                        }
                    } else if (key == "ESTOP") {
                        new_fb.safety.is_estop_active = (val_str == "1");
                    } else if (key == "POWER") {
                        new_fb.safety.is_power_on = (val_str == "1");
                    } else if (key == "MODE") {
                        int mode = 0;
                        std::from_chars(val_str.data(), val_str.data() + val_str.size(), mode);
                        new_fb.execution_mode = mode == static_cast<int>(HalCommandExecutionMode::InternalInterpolator)
                            ? HalCommandExecutionMode::InternalInterpolator
                            : HalCommandExecutionMode::Teleport;
                    } else if (key == "TR") {
                        new_fb.target_reached = (val_str == "1");
                    } else if (key == "MA") {
                        new_fb.motion_active = (val_str == "1");
                    } else if (key.starts_with("HS")) {
                        int axis_idx = 0;
                        std::from_chars(key.data() + 2, key.data() + key.size(), axis_idx);
                        if (axis_idx >= 1 && axis_idx <= ROBOT_AXES_COUNT) {
                            int hs = 0;
                            std::from_chars(val_str.data(), val_str.data() + val_str.size(), hs);
                            // Homing is owned by the backend; we no longer keep a homing state
                            // machine. Reflect "axis is homing now" into the raw axis state
                            // (last_status) using the shared AxisState convention
                            // (Homing == 5), which is all MotionManager::isHomingActive() and
                            // the UI need. hs: 1=Requested, 2=InProgress -> homing in progress.
                            constexpr int kRawStateHoming = 5;
                            constexpr int kRawStateUnknown = 0;
                            std::lock_guard<std::mutex> lock(hal_state_mutex_);
                            hal_state_.axes[static_cast<size_t>(axis_idx - 1)].last_status =
                                (hs == 1 || hs == 2) ? kRawStateHoming : kRawStateUnknown;
                        }
                    }
                }
            }

            for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
                const auto& raw_axis = raw_phys_joints.GetAt(i).value().get();
                auto& logical_axis = new_fb.joints.GetAt(i).value().get();
                logical_axis.position = Degrees(offsets_.toLogical(i, raw_axis.position.value()));
            }
            
            new_fb.driver_status = HalStatus::Ok;
            cached_feedback_.store(new_fb, std::memory_order_release);
            
        } else if (bytes == 0) { // Timeout
            HardwareFeedback current_fb = cached_feedback_.load(std::memory_order_acquire);
            current_fb.driver_status = HalStatus::Warning_SyncLost;
            cached_feedback_.store(current_fb, std::memory_order_release);
            RDT_LOG_WARN(MODULE_NAME, "UDP receive timeout. Status set to SyncLost.");
        } else { // Non-timeout error (e.g. ICMP port-unreachable while the peer is down)
            HardwareFeedback current_fb = cached_feedback_.load(std::memory_order_acquire);
            if (current_fb.driver_status != HalStatus::Error_CommunicationLost) {
                // Log only the transition into the error state, not every retry.
                RDT_LOG_ERROR(MODULE_NAME, "UDP receive error. Status set to CommunicationLost; worker keeps retrying.");
                current_fb.driver_status = HalStatus::Error_CommunicationLost;
                cached_feedback_.store(current_fb, std::memory_order_release);
            }
            // Keep the worker alive: on Windows a send to a closed port surfaces as a receive
            // error (ICMP reset) and is transient once the peer restarts. Previously the worker
            // thread died here permanently, so the link never recovered without a driver
            // reinit. Pace the retry so a hard socket failure cannot busy-spin the thread.
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
    RDT_LOG_INFO(MODULE_NAME, "UDP worker thread finished.");
}

} // namespace RDT