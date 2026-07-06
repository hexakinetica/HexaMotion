#include "HardwareManager.h"
#include "interface/IDriver.h"
#include "drivers/sim/SimDriver.h"
#include "drivers/udp/UdpDriver.h"
#include "drivers/mks_tcp/MksTcpDriver.h"
#include "LoggingMacros.h"
#include <cmath>
#include <thread> // For sleep_for

namespace RDT {

namespace {

// Bounded wait for the first feedback packet after (re)creating the realtime driver. Runs on
// the NRT/controller thread, so a failed connect must not freeze the control loop / status
// broadcast for seconds; the loop exits as soon as the link reports Ok (M1).
constexpr std::chrono::milliseconds kFirstFeedbackTimeout{500};
constexpr std::chrono::milliseconds kFirstFeedbackPollInterval{10};

// Stamps every axis of the state with one status/error pair (connect/disconnect outcome).
void markAllAxes(HalConfigState& state, HalStatus status, int error_code) {
    for (auto& axis : state.axes) {
        axis.last_status = static_cast<int>(status);
        axis.last_error_code = error_code;
    }
}

} // namespace

HardwareManager::HardwareManager(const InterfaceConfig& config, const RobotLimits& limits)
    : config_(config)
    , limits_(limits)
    , realtime_type_(config.realtime_type)
{
    if (realtime_type_.load(std::memory_order_relaxed) == InterfaceConfig::RealtimeInterfaceType::MksTcp) {
        runtime_mks_tcp_ip_ = config_.mks_tcp_config.ip;
        runtime_mks_tcp_port_ = config_.mks_tcp_config.port;
    }
    sim_driver_ = std::make_shared<SimDriver>(config_.simulation_initial_joints, limits_);
    initializeHalState();
}

HardwareManager::~HardwareManager() {
    shutdown();
}

Result<void, ErrorCode> HardwareManager::init() {
    auto sim_res = sim_driver_->init();
    if (sim_res.isError()) {
        RDT_LOG_CRITICAL("HardwareManager", "Sim driver init failed: {}", static_cast<int>(sim_res.error()));
        return sim_res;
    }

    // The HAL always starts in Simulation (active_mode_ member default). The Real driver is
    // initialized for ALL backends: UDP/MKS so keep-alive Hold commands and real telemetry work
    // while the operator is in Simulation view, and the sim backend so the virtual Motor
    // Configurator exists from the start (REQ-SIMMC-01 — full topology parity with mks_tcp).
    RDT_LOG_INFO("HardwareManager", "Initializing Realtime driver (background)...");
    auto res = reinitializeRealtimeDriver();
    if (res.isError()) {
        RDT_LOG_WARN("HardwareManager", "Failed to initialize Realtime driver in background: {}. Keep-alive will be disabled.", static_cast<int>(res.error()));
    } else {
        // reinitializeRealtimeDriver() has already cached the first real feedback; align the
        // simulation so the "Ghost" appears at the real robot's pose from the start.
        (void)syncSimulationToReal();
    }

    RDT_LOG_INFO("HardwareManager", "Started in Simulation mode.");
    return Result<void, ErrorCode>::Success();
}

void HardwareManager::shutdown() {
    if (sim_driver_) sim_driver_->stop();
    resetRealDriver();
}

Result<void, ErrorCode> HardwareManager::write(const AxisSet& cmd) {
    HardwareCommand command{};
    command.stream_target = cmd;
    return write(command);
}

Result<void, ErrorCode> HardwareManager::write(const HardwareCommand& cmd) {
    HardwareCommand safe_command = cmd;
    // The command governor clamps the per-cycle STREAM target (velocity/soft limits). This is
    // effective for stream backends (UDP). For point-to-point backends that ignore the stream
    // and consume only segment_target (MKS, execution_mode==InternalInterpolator), velocity and
    // soft limits are enforced by the Motor Configurator / IPC config, NOT here (decision П3).
    // segment_target is intentionally passed through unclamped; position limits are still checked
    // upstream at enqueue (MotionManager::validateTargetPoint).
    safe_command.stream_target = applyCommandGovernor(cmd.stream_target);
    auto active = getActiveDriverInstance();
    if (!active) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }

    // Synchronous Keep-Alive for Real Robot in Simulation Mode.
    // NOT sent to the virtual MC (sim backend, REQ-SIMMC-04): it has no transport to keep
    // alive, and unlike MksTcpDriver — which ignores pure-stream writes — SimDriver follows
    // them, so a stale Hold would silently undo service ops (zero/homing/jog) on the virtual MC.
    if (active_mode_.load(std::memory_order_relaxed) == HalMode::Simulation &&
        realtime_type_.load(std::memory_order_relaxed) != InterfaceConfig::RealtimeInterfaceType::None) {
        if (auto rd = snapshotRealDriver()) {
            // Send "Hold" command (last known real position) to prevent timeout.
            // This is safe assuming real_driver_->write is non-blocking and fast.
            AxisSet hold;
            {
                std::lock_guard<std::mutex> lock(real_feedback_mutex_);
                hold = real_feedback_.joints;
            }
            (void)rd->write(hold);
        }
    }

    return active->writeCommand(safe_command);
}

Result<HardwareFeedback, ErrorCode> HardwareManager::read() {
    auto active = getActiveDriverInstance();
    if (!active) {
        return Result<HardwareFeedback, ErrorCode>::Failure(ErrorCode::NotConnected);
    }

    auto feedback_res = active->read();
    if (feedback_res.isError()) {
        return feedback_res;
    }
    HardwareFeedback final_feedback = feedback_res.value();

    const HalMode mode = active_mode_.load(std::memory_order_relaxed);
    if (mode == HalMode::Simulation) {
        // Synchronously poll Real Robot status so "Ghost" updates at full RT frequency.
        if (auto rd = snapshotRealDriver()) {
            auto real_res = rd->read();
            if (real_res.isSuccess()) {
                std::lock_guard<std::mutex> lock(real_feedback_mutex_);
                real_feedback_ = real_res.value();
            }
        }
    } else if (mode == HalMode::Realtime) {
        // Update the real feedback cache in Realtime mode too, so switching back to Simulation
        // does not "jump" from a stale cached real position. UI Ghost follows the real robot.
        std::lock_guard<std::mutex> lock(real_feedback_mutex_);
        real_feedback_ = final_feedback;
    }

    {
        // POD-only refresh: the full HalConfigState carries std::string diagnostics whose copy
        // would heap-allocate every RT cycle. The per-cycle mirror only needs the axes slice
        // (governor clamps, homing/fault status); the string-bearing fields keep the values
        // committed by setHalConfig()/initializeHalState().
        const auto axis_state = active->getAxisRuntimeState();
        std::unique_lock lock(hal_config_mutex_);
        hal_config_current_.axes = axis_state;
    }

    return Result<HardwareFeedback, ErrorCode>::Success(final_feedback);
}

Result<void, ErrorCode> HardwareManager::setMode(HalMode mode) {
    if (mode == active_mode_.load()) {
        return Result<void, ErrorCode>::Success();
    }

    // When switching TO Simulation (from Realtime), sync Sim -> Real so the Orange robot
    // (SimDriver) starts exactly where the Real robot is.
    if (mode == HalMode::Simulation && active_mode_.load() == HalMode::Realtime) {
        if (snapshotRealDriver()) {
            (void)syncSimulationToReal();
        }
    }

    if (mode == HalMode::Realtime) {
        // Sim backend included: the virtual MC (REQ-SIMMC-01/-03) goes through the SAME gate as
        // real hardware — pose-sync check, sync-back, honest refusals. This supersedes the
        // 2026-07-06 "switch is immediate on the stub" special case; out of the box both drivers
        // start at simulation_initial_joints, so the first switch still passes immediately.
        if (!snapshotRealDriver()) {
            RDT_LOG_INFO("HardwareManager", "Initializing Real driver on mode switch demand...");
            // reinitializeRealtimeDriver() already waits (bounded, M1) for the first feedback
            // packet, so no additional wait is needed here.
            auto res = reinitializeRealtimeDriver();
            if (res.isError()) {
                return res;
            }
        }

        auto real = snapshotRealDriver();
        if (!real) {
            RDT_LOG_ERROR("HardwareManager", "Cannot switch to REAL: Real driver not initialized.");
            return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
        }
        auto sim_res = sim_driver_->read();
        auto real_res = real->read();
        if (sim_res.isError()) {
            RDT_LOG_ERROR("HardwareManager", "Cannot switch to REAL: Sim driver read failed.");
            return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
        }
        if (real_res.isError()) {
            RDT_LOG_ERROR("HardwareManager", "Cannot switch to REAL: Real driver read failed (Code {}).", static_cast<int>(real_res.error()));
            return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
        }

        if (real_res.value().driver_status == HalStatus::Ok) {
            const auto& sim_joints = sim_res.value().joints;
            const auto& real_joints = real_res.value().joints;
            constexpr double kSyncToleranceDeg = 10.0; // Relaxed tolerance to prevent rejection on minor deviations

            for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
                 auto diff_deg = std::abs(sim_joints.GetAt(i).value().get().position.value() - real_joints.GetAt(i).value().get().position.value());
                 if (diff_deg > kSyncToleranceDeg) {
                     RDT_LOG_ERROR("HardwareManager", "Mode switch rejected! Axis {} sync error: {} deg (Sim={:.3f}, Real={:.3f})",
                         i, diff_deg,
                         sim_joints.GetAt(i).value().get().position.value(),
                         real_joints.GetAt(i).value().get().position.value());
                     return Result<void, ErrorCode>::Failure(ErrorCode::InvalidArgument);
                 }
            }
        } else {
            // Fail-closed (boss decision): entering REAL with a link that is not Ok would skip
            // the pose-sync gate and hand the operator a mode that cannot actually drive the
            // robot. reinitializeRealtimeDriver() already waited (bounded) for the first packet,
            // so a not-Ok status here means the backend is genuinely unreachable.
            RDT_LOG_ERROR("HardwareManager",
                          "Cannot switch to REAL: driver link is not Ok (Status {}). Staying in Simulation.",
                          static_cast<int>(real_res.value().driver_status));
            return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
        }

        // After a successful sync check, force the simulation to adopt the real robot's state.
        // This handles cases where the real robot might have drifted slightly.
        (void)syncSimulationToReal();
    }

    active_mode_.store(mode);

    is_first_cmd_.store(true); // Reset governor
    RDT_LOG_INFO("HardwareManager", "Mode set to {}.", (mode == HalMode::Realtime ? "REAL" : "SIMULATION"));
    return Result<void, ErrorCode>::Success();
}

HalMode HardwareManager::getMode() const {
    return active_mode_.load(std::memory_order_relaxed);
}

Result<void, ErrorCode> HardwareManager::setRealtimeInterfaceType(InterfaceConfig::RealtimeInterfaceType type) {
    const auto current = realtime_type_.load(std::memory_order_relaxed);
    if (type == current) {
        return Result<void, ErrorCode>::Success();   // no change; idempotent
    }
    // Fail-closed: never swap the hardware backend while the Realtime mode drives it. The
    // operator must switch the application to Simulation view first (explicit, traceable).
    if (active_mode_.load(std::memory_order_relaxed) == HalMode::Realtime) {
        RDT_LOG_WARN("HardwareManager",
                     "Realtime backend switch to type {} REFUSED: Realtime mode is active. "
                     "Switch to Simulation view first.",
                     static_cast<int>(type));
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidState);
    }

    RDT_LOG_INFO("HardwareManager", "Realtime backend switch: type {} -> {}.",
                 static_cast<int>(current), static_cast<int>(type));

    // Tear down the old realtime driver (safe: we are not in Realtime mode) and re-seed the
    // HAL runtime state for the new backend. For MKS the transport comes up later through the
    // explicit CONNECT command; for Sim there is no transport — the virtual MC is created
    // eagerly right here so the ghost/panel are live immediately (REQ-SIMMC-01).
    resetRealDriver();
    realtime_type_.store(type, std::memory_order_relaxed);
    initializeHalState();
    if (type == InterfaceConfig::RealtimeInterfaceType::None) {
        // No sim-pose sync here, mirroring connectTransport(): the operator sees the ghost at the
        // virtual MC's home pose and reconciles explicitly (jog/homing) before SIM->REAL.
        auto reinit_res = reinitializeRealtimeDriver();
        if (reinit_res.isError()) {
            return reinit_res;
        }
    }
    return Result<void, ErrorCode>::Success();
}

InterfaceConfig::RealtimeInterfaceType HardwareManager::getRealtimeInterfaceType() const {
    return realtime_type_.load(std::memory_order_relaxed);
}

Result<void, ErrorCode> HardwareManager::setHalConfig(const HalConfigCommand& cmd) {
    HalConfigState current_state;
    {
        std::shared_lock lock(hal_config_mutex_);
        current_state = hal_config_current_;
    }

    // Build the desired state: current state plus the command's per-axis updates and
    // transport endpoint overrides.
    HalConfigState desired_state = current_state;
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        applyAxisCommandToState(i, cmd.axes[i], desired_state);
    }
    desired_state.appliedRequestId = cmd.requestId;
    if (!cmd.transport_ip.empty()) {
        desired_state.transport_ip = cmd.transport_ip;
    }
    if (cmd.transport_port > 0) {
        desired_state.transport_port = cmd.transport_port;
    }

    switch (cmd.transport_command) {
    case HalTransportCommand::Disconnect:
        return disconnectTransport(std::move(desired_state));
    case HalTransportCommand::Connect:
        return connectTransport(cmd.requestId, std::move(desired_state));
    case HalTransportCommand::None:
        return applyConfigToDrivers(cmd.requestId, current_state, std::move(desired_state));
    }

    RDT_LOG_ERROR("HardwareManager", "Unknown HAL transport command {} in request {}.",
                  static_cast<int>(cmd.transport_command), cmd.requestId);
    return Result<void, ErrorCode>::Failure(ErrorCode::InvalidArgument);
}

Result<void, ErrorCode> HardwareManager::disconnectTransport(HalConfigState desired_state) {
    resetRealDriver();

    desired_state.transport_connected = false;
    markAllAxes(desired_state, HalStatus::NotConnected, 0);
    commitHalState(desired_state);

    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> HardwareManager::connectTransport(uint32_t request_id, HalConfigState desired_state) {
    if (realtime_type_.load(std::memory_order_relaxed) != InterfaceConfig::RealtimeInterfaceType::MksTcp) {
        RDT_LOG_WARN("HardwareManager",
                     "HAL transport connect request {} is unsupported for realtime_type {}.",
                     request_id,
                     static_cast<int>(realtime_type_.load(std::memory_order_relaxed)));
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidArgument);
    }

    runtime_mks_tcp_ip_ = desired_state.transport_ip.empty() ? std::string(kDefaultMksTcpIp) : desired_state.transport_ip;
    runtime_mks_tcp_port_ = desired_state.transport_port > 0 ? desired_state.transport_port : kDefaultMksTcpPort;

    resetRealDriver();

    auto reinit_res = reinitializeRealtimeDriver();
    if (reinit_res.isError()) {
        desired_state.transport_connected = false;
        markAllAxes(desired_state, HalStatus::Error_CommunicationLost, static_cast<int>(reinit_res.error()));
        commitHalState(desired_state);
        return reinit_res;
    }

    desired_state.transport_connected = true;
    auto rd = snapshotRealDriver();
    auto forward_res = rd
        ? forwardHalConfig(*rd, "realtime", desired_state, request_id)
        : Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    if (forward_res.isError()) {
        desired_state.transport_connected = false;
        markAllAxes(desired_state, HalStatus::Error_CommunicationLost, static_cast<int>(forward_res.error()));
        commitHalState(desired_state);
        return forward_res;
    }

    commitHalState(desired_state);
    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> HardwareManager::applyConfigToDrivers(uint32_t request_id,
                                                              const HalConfigState& current_state,
                                                              HalConfigState desired_state) {
    // On a sim-only bench (realtime_type == None) there is no hardware side by configuration:
    // its absence is not a communication error and must not poison the per-axis status.
    const bool realtime_expected = realtime_type_.load(std::memory_order_relaxed) != InterfaceConfig::RealtimeInterfaceType::None;

    auto active_result = Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    auto active = getActiveDriverInstance();
    if (active) {
        active_result = forwardHalConfig(*active,
            active.get() == sim_driver_.get() ? "simulation" : "active realtime",
            desired_state, request_id);
    }

    // The realtime driver receives the config even when Simulation is active. If it dropped,
    // try to bring it back once so the config lands on the hardware side too.
    if (!snapshotRealDriver() && realtime_expected) {
        RDT_LOG_INFO("HardwareManager",
                     "Realtime driver unavailable during HAL config request {}. Attempting reinitialize.",
                     request_id);
        auto reinit_res = reinitializeRealtimeDriver();
        if (reinit_res.isError()) {
            RDT_LOG_WARN("HardwareManager",
                         "Realtime driver reinitialize failed during HAL config request {}. Error={}",
                         request_id,
                         static_cast<int>(reinit_res.error()));
        }
    }

    auto realtime_result = Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    auto rd = snapshotRealDriver();
    active = getActiveDriverInstance();
    if (rd && rd.get() != active.get()) {
        realtime_result = forwardHalConfig(*rd, "realtime", desired_state, request_id);
    } else if (rd && rd.get() == active.get()) {
        realtime_result = active_result;
    }

    const bool any_success = active_result.isSuccess() || realtime_result.isSuccess();

    HalConfigState committed_state = any_success ? desired_state : current_state;
    markAllAxes(committed_state,
                any_success ? HalStatus::Ok : HalStatus::Error_CommunicationLost,
                any_success ? 0 : static_cast<int>(ErrorCode::NotConnected));

    // A partial failure (one driver took the config, the other did not) is still surfaced
    // per-axis so the operator sees the communication problem. The realtime leg counts only
    // when a realtime backend is configured at all.
    if (active_result.isError() || (realtime_expected && realtime_result.isError())) {
        const int error_code = active_result.isError()
            ? static_cast<int>(active_result.error())
            : static_cast<int>(realtime_result.error());
        markAllAxes(committed_state, HalStatus::Error_CommunicationLost, error_code);
    }

    committed_state.appliedRequestId = any_success ? request_id : current_state.appliedRequestId;
    commitHalState(committed_state);

    return any_success
        ? Result<void, ErrorCode>::Success()
        : Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
}

Result<void, ErrorCode> HardwareManager::forwardHalConfig(IDriver& driver, const char* driver_name,
                                                          const HalConfigState& state, uint32_t request_id) {
    auto res = driver.setHalConfig(state);
    if (res.isError()) {
        RDT_LOG_WARN("HardwareManager",
                     "Failed to forward HAL config request {} to {} driver. Error={}",
                     request_id,
                     driver_name,
                     static_cast<int>(res.error()));
    } else {
        RDT_LOG_INFO("HardwareManager",
                     "Forwarded HAL config request {} to {} driver.",
                     request_id,
                     driver_name);
    }
    return res;
}

void HardwareManager::commitHalState(const HalConfigState& state) {
    std::unique_lock lock(hal_config_mutex_);
    hal_config_current_ = state;
}

Result<void, ErrorCode> HardwareManager::setDigitalOutput(uint16_t port, bool state) {
    // Program SET DO follows the MOTION routing (active mode), not the real-first service routing:
    // a SIM dry-run must exercise the simulator's DO/DI loopback, never touch real outputs.
    auto active = getActiveDriverInstance();
    if (!active) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }
    return active->setDigitalOutput(port, state);
}

Result<void, ErrorCode> HardwareManager::requestHoming(int axis_id) {
    // Homing is a real-hardware concept owned by the Motor Configurator; the real-first routing
    // of serviceTargetDriver() makes HOME work from the HAL panel even in Simulation view (the
    // internal sim never homed anyway).
    auto target = serviceTargetDriver();
    if (!target) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }

    RDT_LOG_INFO("HardwareManager",
                 "Forwarding homing request. mode={}, driver={}, axis_id={}",
                 active_mode_.load(std::memory_order_relaxed) == HalMode::Realtime ? "REAL" : "SIM",
                 target.get() == sim_driver_.get() ? "simulation" : "realtime",
                 axis_id);

    return target->requestHoming(axis_id);
}

Result<void, ErrorCode> HardwareManager::clearDriveErrors(int axis_id) {
    // Faults are a real-hardware concept: real-first routing makes Clear Error work even if we
    // are momentarily in Simulation mode; the sim driver's default no-op keeps this harmless.
    auto target = serviceTargetDriver();
    if (!target) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }
    RDT_LOG_INFO("HardwareManager", "Forwarding clear-drive-errors request. axis_id={}", axis_id);
    return target->clearDriveErrors(axis_id);
}

HalConfigState HardwareManager::getCurrentHalState() const {
    // Intentional source-of-truth split (decision П5): the real driver is preferred whenever it
    // exists, even in Simulation view, because homing/fault state is a real-hardware concept that
    // HexaMotion forwards to the Motor Configurator and reads back (e.g. MotionManager::isHomingActive).
    // The cached hal_config_current_ (populated from the *active* driver in read()) is only the
    // fallback when no real driver is present, and feeds applyCommandGovernor — which is bypassed in
    // Simulation anyway, so the two sources never disagree in a way that affects motion.
    if (auto rd = snapshotRealDriver()) {
        return rd->getCurrentHalState();
    }

    std::shared_lock lock(hal_config_mutex_);
    return hal_config_current_;
}

Result<void, ErrorCode> HardwareManager::reinitializeRealtimeDriver() {
    // Build and initialize the new driver on a LOCAL handle, with real_driver_mutex_ released,
    // so the (potentially blocking) connect/handshake never stalls the RT loop. Only the final
    // pointer publish takes the lock (C1).
    std::shared_ptr<IDriver> new_driver;
    switch (realtime_type_.load(std::memory_order_relaxed)) {
    case InterfaceConfig::RealtimeInterfaceType::None:
        // Sim backend: the realtime driver is a VIRTUAL Motor Configurator — a dedicated
        // SimDriver instance playing the physical-hardware role (REQ-SIMMC-01), so the ghost,
        // link status, mode gate and service commands run the same code as with real hardware.
        new_driver = std::make_shared<SimDriver>(config_.simulation_initial_joints, limits_);
        break;
    case InterfaceConfig::RealtimeInterfaceType::Udp:
        new_driver = std::make_shared<UdpDriver>(config_.udp_control_config);
        break;
    case InterfaceConfig::RealtimeInterfaceType::MksTcp: {
        MksTcpConfig runtime_cfg = config_.mks_tcp_config;
        if (!runtime_mks_tcp_ip_.empty()) {
            runtime_cfg.ip = runtime_mks_tcp_ip_;
        }
        if (runtime_mks_tcp_port_ > 0) {
            runtime_cfg.port = runtime_mks_tcp_port_;
        }
        new_driver = std::make_shared<MksTcpDriver>(runtime_cfg);
        break;
    }
    default:
        RDT_LOG_WARN("HardwareManager",
                     "Cannot init Realtime driver: no driver for realtime_type {}.",
                     static_cast<int>(realtime_type_.load(std::memory_order_relaxed)));
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidArgument);
    }

    auto res = new_driver->init();
    if (res.isError()) {
        RDT_LOG_ERROR("HardwareManager", "Real driver re-init failed: {}", static_cast<int>(res.error()));
        return res;
    }

    // Publish the ready driver atomically.
    {
        std::lock_guard<std::mutex> lock(real_driver_mutex_);
        real_driver_ = new_driver;
    }

    RDT_LOG_INFO("HardwareManager", "Real driver re-initialized successfully.");

    // Read initial state for keep-alive. Wait briefly (bounded, M1) for the first packet —
    // UDP/MKS report NotConnected until it arrives.
    auto start_wait = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_wait < kFirstFeedbackTimeout) {
        auto read_res = new_driver->read();
        if (read_res.isSuccess() && read_res.value().driver_status == HalStatus::Ok) {
            std::lock_guard<std::mutex> lock(real_feedback_mutex_);
            real_feedback_ = read_res.value();
            RDT_LOG_INFO("HardwareManager", "Real driver ready. Initial position acquired.");
            return Result<void, ErrorCode>::Success();
        }
        std::this_thread::sleep_for(kFirstFeedbackPollInterval);
    }

    // [ROBUSTNESS] If timeout, do NOT leave the real feedback cache as zeroes.
    // Initialize it from the SimDriver (which is at Home/Initial Config) so a late-connecting
    // external HAL receives "Home" as the target, not "Zero".
    RDT_LOG_WARN("HardwareManager", "Real driver initialized but no data received within timeout. Using Simulation state as initial Keep-Alive.");

    if (sim_driver_) {
        auto sim_res = sim_driver_->read();
        if (sim_res.isSuccess()) {
             std::lock_guard<std::mutex> lock(real_feedback_mutex_);
             real_feedback_ = sim_res.value();
             // Mark as NotConnected so we know it's synthetic
             real_feedback_.driver_status = HalStatus::NotConnected;
        }
    }

    return Result<void, ErrorCode>::Success();
}

Result<void, ErrorCode> HardwareManager::syncSimulationToReal() {
    auto rd = snapshotRealDriver();
    if (!rd || !sim_driver_) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }
    auto res = rd->read();
    if (res.isSuccess()) {
        (void)sim_driver_->setState(res.value().joints);
        return Result<void, ErrorCode>::Success();
    }
    return Result<void, ErrorCode>::Failure(res.error());
}

Result<HardwareFeedback, ErrorCode> HardwareManager::getRealDriverFeedback() {
    if (!snapshotRealDriver()) {
        return Result<HardwareFeedback, ErrorCode>::Failure(ErrorCode::NotConnected);
    }
    // Return clean cached feedback protected by mutex (avoids a direct read racing the RT loop).
    std::lock_guard<std::mutex> lock(real_feedback_mutex_);
    return Result<HardwareFeedback, ErrorCode>::Success(real_feedback_);
}

Result<void, ErrorCode> HardwareManager::zeroAxis(AxisId axis) {
    return masterAxisAt(axis, Degrees(0.0));
}

Result<void, ErrorCode> HardwareManager::masterAxisAt(AxisId axis, Degrees logical_position) {
    // Set-Zero is a hardware calibration op; real-first routing makes it work from the HAL
    // panel in Simulation view too.
    auto target_driver = serviceTargetDriver();
    if (!target_driver) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }
    auto res = target_driver->masterAxisAt(axis, logical_position);
    if (res.isSuccess()) {
        is_first_cmd_.store(true);
    }
    return res;
}

Result<void, ErrorCode> HardwareManager::zeroAllAxes() {
    // Same routing as masterAxisAt, but a single atomic "zero all" so the per-axis requests do not
    // collapse upstream. The MKS driver sends one SetZeroAll; other drivers loop per-axis (default).
    auto target_driver = serviceTargetDriver();
    if (!target_driver) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }
    auto res = target_driver->masterAllAxes();
    if (res.isSuccess()) {
        is_first_cmd_.store(true);
    }
    return res;
}

Result<void, ErrorCode> HardwareManager::jogRealIncremental(int axis, double delta_deg, double speed_ratio) {
    // REAL-driver-only by design; on the sim backend the real driver is the virtual MC
    // (REQ-SIMMC-06), which is safe to jog in SIM view because the keep-alive Hold stream is
    // not sent to it (REQ-SIMMC-04). In REAL view on the sim backend the controller routes the
    // HAL jog through the planner instead: there the virtual MC is the ACTIVE stream-followed
    // driver and a point-to-point jump would be overwritten by the 4 ms hold stream.
    auto rd = snapshotRealDriver();
    if (!rd) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }
    if (axis < 0 || axis >= static_cast<int>(ROBOT_AXES_COUNT)) {
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidAxisId);
    }
    // Read the MC's current pose and send a point-to-point endpoint with the jogged axis nudged.
    // The MKS firmware profiles to the endpoint itself, so this single segment IS the jog.
    auto fb = rd->read();
    if (fb.isError()) {
        return Result<void, ErrorCode>::Failure(fb.error());
    }

    if (realtime_type_.load(std::memory_order_relaxed) == InterfaceConfig::RealtimeInterfaceType::None) {
        // Virtual MC (sim backend): apply the jog via setState — instant arrival at the nudged
        // LOGICAL target, same convention as simulated homing; speed_ratio is not modelled.
        // Deliberately NOT via writeCommand/segment_target: SimDriver is a pure stream follower
        // (MotionManager tags every streamed point with segment metadata for the MKS firmware,
        // so a segment-consuming SimDriver teleported the ACTIVE robot to the segment endpoint
        // mid-move and tripped Error_FollowingError on long moves — 100 mm regression,
        // 2026-07-07, REQ-SIMMC-05).
        AxisSet target = fb.value().joints;
        auto& axis_state = target.GetAt(static_cast<size_t>(axis)).value().get();
        axis_state.position = Degrees(axis_state.position.value() + delta_deg);
        return rd->setState(target);
    }

    HardwareCommand cmd{};
    cmd.segment_target.start_angles = fb.value().joints.ToPositionArray();
    cmd.segment_target.target_angles = cmd.segment_target.start_angles;
    const auto idx = static_cast<size_t>(axis);
    cmd.segment_target.target_angles[idx] = Degrees(cmd.segment_target.target_angles[idx].value() + delta_deg);
    // Carry the UI SPEED% so the MKS driver scales the axis speed for this HAL-direct jog.
    cmd.segment_target.speed_ratio = speed_ratio;
    cmd.has_segment_target = true;
    return rd->writeCommand(cmd);
}

Result<void, ErrorCode> HardwareManager::emergencyStopAll() {
    // Real-first routing (same as requestHoming): the physical drives must stop even when the
    // operator is in Simulation view. Falls back to the active driver's default no-op when no
    // real driver exists (sim is halted by MotionManager::emergencyStop()).
    auto target = serviceTargetDriver();
    if (!target) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }
    RDT_LOG_CRITICAL("HardwareManager", "Forwarding emergency stop to {} driver.",
                     target.get() == sim_driver_.get() ? "active" : "realtime");
    return target->emergencyStopAll();
}

Result<void, ErrorCode> HardwareManager::setBrakeState(bool engaged) {
    auto target_driver = getActiveDriverInstance();
    if (!target_driver) {
        return Result<void, ErrorCode>::Failure(ErrorCode::NotConnected);
    }

    auto res = target_driver->setBrakeState(engaged);
    if (res.isSuccess()) {
        is_first_cmd_.store(true);
    }
    return res;
}

void HardwareManager::initializeHalState() {
    std::unique_lock lock(hal_config_mutex_);
    hal_config_current_ = {};
    if (realtime_type_.load(std::memory_order_relaxed) == InterfaceConfig::RealtimeInterfaceType::MksTcp) {
        hal_config_current_.transport_ip = runtime_mks_tcp_ip_.empty() ? std::string(kDefaultMksTcpIp) : runtime_mks_tcp_ip_;
        hal_config_current_.transport_port = runtime_mks_tcp_port_ > 0 ? runtime_mks_tcp_port_ : kDefaultMksTcpPort;
    }
    hal_config_current_.transport_connected = false;

    // Sim backend (realtime_type == None): motors are enabled out of the box (boss directive
    // 2026-07-06) — there is no physical drive to protect, and the HAL panel must be usable
    // for debugging immediately. Real backends keep the fail-closed disabled default.
    const bool sim_backend = (realtime_type_.load(std::memory_order_relaxed) == InterfaceConfig::RealtimeInterfaceType::None);

    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        hal_config_current_.axes[i].motor_enabled = sim_backend;
        hal_config_current_.axes[i].soft_limits_enabled = true;
        hal_config_current_.axes[i].soft_limit_min = limits_.joint_position_limits_deg[i].first;
        hal_config_current_.axes[i].soft_limit_max = limits_.joint_position_limits_deg[i].second;
        hal_config_current_.axes[i].velocity_limit_enabled = true;
        hal_config_current_.axes[i].velocity_limit = limits_.joint_velocity_limits_deg_s[i];
        hal_config_current_.axes[i].last_status = static_cast<int>(HalStatus::Ok);
        hal_config_current_.axes[i].last_error_code = 0;
    }
}

void HardwareManager::applyAxisCommandToState(size_t axis_index, const HalAxisConfigCommand& axis_cmd, HalConfigState& state) const {
    auto& axis_state = state.axes[axis_index];

    if (axis_cmd.update_mask & HalConfigUpdate_MotorEnabled) {
        axis_state.motor_enabled = axis_cmd.motor_enabled;
    }
    if (axis_cmd.update_mask & HalConfigUpdate_SoftLimits) {
        axis_state.soft_limits_enabled = axis_cmd.soft_limits_enabled;
        axis_state.soft_limit_min = axis_cmd.soft_limit_min;
        axis_state.soft_limit_max = axis_cmd.soft_limit_max;
    }
    if (axis_cmd.update_mask & HalConfigUpdate_VelocityLimit) {
        axis_state.velocity_limit_enabled = axis_cmd.velocity_limit_enabled;
        axis_state.velocity_limit = axis_cmd.velocity_limit;
    }

    axis_state.last_status = static_cast<int>(HalStatus::Ok);
    axis_state.last_error_code = 0;
}

std::shared_ptr<IDriver> HardwareManager::getActiveDriverInstance() {
    if (active_mode_.load(std::memory_order_relaxed) == HalMode::Realtime) {
        // ALL backends, sim included (virtual MC, REQ-SIMMC-01): a missing driver in REAL mode
        // stays a null return — an honest fault the RT loop surfaces as an error, never silently
        // downgraded to simulation.
        return snapshotRealDriver();
    }
    return sim_driver_;
}

std::shared_ptr<IDriver> HardwareManager::snapshotRealDriver() const {
    std::lock_guard<std::mutex> lock(real_driver_mutex_);
    return real_driver_;
}

std::shared_ptr<IDriver> HardwareManager::serviceTargetDriver() {
    auto rd = snapshotRealDriver();
    return rd ? rd : getActiveDriverInstance();
}

void HardwareManager::resetRealDriver() {
    std::shared_ptr<IDriver> old;
    {
        std::lock_guard<std::mutex> lock(real_driver_mutex_);
        old.swap(real_driver_);
    }
    if (old) {
        old->stop(); // stop() may block on the socket; do it with the lock released.
    }
}

AxisSet HardwareManager::applyCommandGovernor(const AxisSet& target) {
    auto now = std::chrono::steady_clock::now();

    // Internal simulation is a trajectory sandbox and must not depend on
    // runtime HAL drive state such as motor enables, brakes or live clamps.
    if (active_mode_.load(std::memory_order_relaxed) == HalMode::Simulation) {
        last_sent_cmd_ = target;
        last_cmd_time_ = now;
        is_first_cmd_.store(false);
        return target;
    }

    if (is_first_cmd_.load()) {
        last_sent_cmd_ = target;
        last_cmd_time_ = now;
        is_first_cmd_.store(false);
        governor_clamp_logged_ = false;   // new motion burst: re-arm the one-shot clamp warning
        return target;
    }

    double dt_s = std::chrono::duration<double>(now - last_cmd_time_).count();
    if (dt_s <= 1e-6) {
        return last_sent_cmd_;
    }

    AxisSet validated = last_sent_cmd_;
    // Axes-only snapshot (POD): copying the full HalConfigState would copy its std::string
    // diagnostics on the RT write path.
    std::array<HalAxisConfigState, ROBOT_AXES_COUNT> axes_snapshot;
    {
        std::shared_lock lock(hal_config_mutex_);
        axes_snapshot = hal_config_current_.axes;
    }

    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
         auto& current_axis_validated = validated.GetAt(i).value().get();
         const auto& last_sent_axis = last_sent_cmd_.GetAt(i).value().get();
         const auto& target_axis = target.GetAt(i).value().get();
         const auto& hal_axis = axes_snapshot[i];

         if (!hal_axis.motor_enabled) {
             current_axis_validated.position = last_sent_axis.position;
             continue;
         }

         Degrees requested_position = target_axis.position;
         if (hal_axis.soft_limits_enabled) {
             requested_position = Degrees(std::clamp(requested_position.value(),
                                                    hal_axis.soft_limit_min.value(),
                                                    hal_axis.soft_limit_max.value()));
         }

         double delta = requested_position.value() - last_sent_axis.position.value();
         double limit_vel = hal_axis.velocity_limit_enabled
             ? hal_axis.velocity_limit.value()
             : limits_.joint_velocity_limits_deg_s[i].value();

         if (std::abs(delta) / dt_s > limit_vel) {
             if (!governor_clamp_logged_) {
                 // One-shot per motion burst (RT path): the clamp makes the actual lag the
                 // planner stream, which surfaces downstream as Error_FollowingError — this
                 // warning names the real cause and the offending limit.
                 RDT_LOG_WARN("HardwareManager",
                              "Command governor clamping axis {}: requested {:.1f} deg/s exceeds the HAL "
                              "velocity limit {:.1f} deg/s. The stream will lag the planner until the "
                              "demand drops (possible following-error).",
                              i + 1, std::abs(delta) / dt_s, limit_vel);
                 governor_clamp_logged_ = true;
             }
             double max_delta = limit_vel * dt_s;
             current_axis_validated.position = Degrees(last_sent_axis.position.value() + ((delta > 0) ? max_delta : -max_delta));
         } else {
             current_axis_validated.position = requested_position;
         }
    }

    last_sent_cmd_ = validated;
    last_cmd_time_ = now;
    return validated;
}

} // namespace RDT
