#include "UdpDriver.h"
#include "LoggingMacros.h"
#include <sstream>
#include <vector>
#include <charconv>

namespace RDT {

UdpDriver::UdpDriver(const UdpConfig& config) : config_(config) {
    for (int i = 0; i < ROBOT_AXES_COUNT; ++i) {
        offsets_[i].store(0.0, std::memory_order_relaxed);
    }
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
    if (!is_running_.load()) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }
    
    AxisSet phys_cmd = cmd;
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        double off = offsets_[i].load(std::memory_order_relaxed);
        auto& axis = phys_cmd.GetAt(i).value().get();
        axis.position = Degrees(axis.position.value() - off);
    }

    // New Protocol Format: "[CMD] A1=10.0000,A2=20.0000,..."
    std::stringstream oss;
    oss.precision(4);
    oss << std::fixed;
    oss << "[CMD]";
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        oss << " A" << (i + 1) << "=" << phys_cmd.GetAt(i).value().get().position.value();
    }
    
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

Result<void, ErrorCode> UdpDriver::resetAxisToZero(AxisId axis, double new_zero_deg) {
    int idx = AxisIdToInt(axis);
    if (idx < 0 || idx >= ROBOT_AXES_COUNT) {
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidAxisId);
    }

    HardwareFeedback current_fb = cached_feedback_.load(std::memory_order_acquire);
    double current_logical_deg = current_fb.joints.GetAt(idx).value().get().position.value();
    double current_offset_deg = offsets_[idx].load(std::memory_order_relaxed);
    
    double current_physical_deg = current_logical_deg - current_offset_deg;
    double new_offset_deg = new_zero_deg - current_physical_deg;
    
    offsets_[idx].store(new_offset_deg);
    
    RDT_LOG_INFO(MODULE_NAME, "Axis {} mastered. Physical: {:.3f} deg, New Offset: {:.3f} deg.", idx, current_physical_deg, new_offset_deg);
    return Result<void, ErrorCode>::Success();
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
                if (msg[start_pos] == ' ') {
                    start_pos++;
                    continue;
                }
                size_t comma_pos = msg.find(',', start_pos);
                if (comma_pos == std::string_view::npos) comma_pos = msg.length();
                std::string_view token = msg.substr(start_pos, comma_pos - start_pos);
                start_pos = comma_pos + 1;
                
                size_t eq_pos = token.find('=');
                if (eq_pos != std::string_view::npos) {
                    std::string_view key = token.substr(0, eq_pos);
                    std::string_view val_str = token.substr(eq_pos + 1);
                    
                    if (key.starts_with("A")) {
                        int axis_idx = key[1] - '1';
                        if (axis_idx >= 0 && axis_idx < ROBOT_AXES_COUNT) {
                            double val_deg = 0.0;
                            std::from_chars(val_str.data(), val_str.data() + val_str.size(), val_deg);
                            raw_phys_joints.GetAt(axis_idx).value().get().position = Degrees(val_deg);
                        }
                    } else if (key.starts_with("DI")) {
                        int di_idx = 0;
                        std::from_chars(key.data() + 2, key.data() + key.size(), di_idx);
                        if (val_str == "1") {
                            new_fb.digital_inputs |= (1 << (di_idx - 1));
                        }
                    } else if (key == "ESTOP") {
                        new_fb.safety.is_estop_active = (val_str == "1");
                    } else if (key == "POWER") {
                        new_fb.safety.is_power_on = (val_str == "1");
                    }
                }
            }

            for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
                double off = offsets_[i].load(std::memory_order_relaxed);
                const auto& raw_axis = raw_phys_joints.GetAt(i).value().get();
                auto& logical_axis = new_fb.joints.GetAt(i).value().get();
                logical_axis.position = Degrees(raw_axis.position.value() + off);
            }
            
            new_fb.driver_status = HalStatus::Ok;
            cached_feedback_.store(new_fb, std::memory_order_release);
            
        } else if (bytes == 0) { // Timeout
            HardwareFeedback current_fb = cached_feedback_.load(std::memory_order_acquire);
            current_fb.driver_status = HalStatus::Warning_SyncLost;
            cached_feedback_.store(current_fb, std::memory_order_release);
            RDT_LOG_WARN(MODULE_NAME, "UDP receive timeout. Status set to SyncLost.");
        } else { // Error
            HardwareFeedback current_fb = cached_feedback_.load(std::memory_order_acquire);
            current_fb.driver_status = HalStatus::Error_CommunicationLost;
            cached_feedback_.store(current_fb, std::memory_order_release);
            RDT_LOG_ERROR(MODULE_NAME, "UDP receive error. Status set to CommunicationLost.");
            break; 
        }
    }
    RDT_LOG_INFO(MODULE_NAME, "UDP worker thread finished.");
}

} // namespace RDT