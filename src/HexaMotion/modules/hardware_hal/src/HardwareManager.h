// HardwareManager.h
#pragma once

#include "HalTypes.h"
#include "RobotConfig.h"
#include "result.h"
#include "ErrorCode.h"
#include <memory>
#include <atomic>
#include <chrono>
#include <mutex>
#include <shared_mutex>

namespace RDT {

class IDriver;

/**
 * @class HardwareManager
 * @brief The public Facade and Guardian of the Hardware Abstraction Layer.
 *
 * The HAL always starts in Simulation mode; switching to Realtime is an explicit,
 * validated operator action via setMode().
 */
class HardwareManager {
public:
    /** @brief Builds the HAL facade over the configured drivers; hardware is opened later by init(). */
    explicit HardwareManager(const InterfaceConfig& config, const RobotLimits& limits);
    // write()/read() are virtual so tests can subclass HardwareManager to inject HAL failures
    // (FailingHardwareManager); a polymorphic base therefore needs a virtual destructor.
    virtual ~HardwareManager();

    HardwareManager(const HardwareManager&) = delete;
    HardwareManager& operator=(const HardwareManager&) = delete;

    /** @brief Creates the sim/real drivers and starts the HAL; call before any write()/read(). */
    [[nodiscard]] Result<void, ErrorCode> init();
    /** @brief Stops the drivers and releases HAL resources. */
    void shutdown();

    /** @brief Convenience overload: wraps the target positions into a HardwareCommand (stream only). */
    [[nodiscard]] Result<void, ErrorCode> write(const AxisSet& cmd);
    /** @brief Sends a full HAL motion command (stream point plus optional segment target) to the active driver; the stream target is clamped by the Command Governor first. */
    [[nodiscard]] virtual Result<void, ErrorCode> write(const HardwareCommand& cmd);
    /**
     * @brief Returns the latest feedback from the active driver.
     * Non-blocking (drivers serve a cached snapshot), but takes short internal locks to
     * refresh the HAL state snapshot — it is not literally wait-free.
     */
    [[nodiscard]] virtual Result<HardwareFeedback, ErrorCode> read();

    /** @brief Switches HAL between Simulation and Realtime; Sim->Realtime is allowed only when sim and real poses match. */
    [[nodiscard]] Result<void, ErrorCode> setMode(HalMode mode);
    /** @brief Returns the currently active HAL mode (Simulation or Realtime). */
    [[nodiscard]] HalMode getMode() const;

    /**
     * @brief Reads feedback specifically from the Realtime driver, even if not active.
     * Useful for monitoring the real robot while in simulation mode.
     */
    Result<HardwareFeedback, ErrorCode> getRealDriverFeedback();

    /** @brief Tears down and recreates the realtime driver (e.g. after an IP/port change or a dropped connection). */
    Result<void, ErrorCode> reinitializeRealtimeDriver();
    /** @brief Copies the real robot's logical pose into the simulation so a later Sim->Realtime switch is in sync. */
    Result<void, ErrorCode> syncSimulationToReal();

    /** @brief Sets the logical zero of one axis by recomputing its offset (Logical = Physical + Offset); no motion. */
    Result<void, ErrorCode> zeroAxis(AxisId axis);

    /** @brief Zero ALL axes in one shot (atomic SetZeroAll on the real driver). */
    Result<void, ErrorCode> zeroAllAxes();

    /** @brief Recomputes one axis' offset so its current physical position reads as the given logical value. */
    Result<void, ErrorCode> masterAxisAt(AxisId axis, Degrees logical_position);

    /**
     * @brief Direct point-to-point jog of one axis on the REAL driver (Motor Configurator),
     * independent of the active SIM/REAL mode and the planner. Used by the HAL overlay so the
     * panel drives the MC even in Simulation view.
     */
    Result<void, ErrorCode> jogRealIncremental(int axis, double delta_deg, double speed_ratio);

    /** @brief Engages or releases the holding brakes on the active driver. */
    Result<void, ErrorCode> setBrakeState(bool engaged);

    /**
     * @brief Drive one digital output on the ACTIVE-mode backend (program SET DO step).
     * Routed like motion: SIM mode -> internal simulator (with DI loopback for dry runs),
     * REAL mode -> the real driver (typed refusal if the backend has no DO channel).
     */
    Result<void, ErrorCode> setDigitalOutput(uint16_t port, bool state);

    /**
     * @brief Forward an emergency stop to the backend hardware (real driver first).
     * The MKS backend profiles motion in firmware, so freezing our own RT stream is not
     * enough — the drives must be actively stopped (audit F-13). Subject to the backend
     * ownership gate (nothing bypasses it by project decision).
     */
    Result<void, ErrorCode> emergencyStopAll();

    /**
     * @brief Live realtime-backend switch (HAL panel dropdown): Sim (None) <-> UDP <-> MKS-TCP.
     * Call from the controller NRT thread only (same thread as setHalConfig/requestHoming).
     * Fail-closed: REFUSED while the Realtime mode is active (the operator must switch the
     * application to Simulation view first) — swapping the hardware backend under a live RT
     * stream would be an untraceable hazard. On success the realtime driver is torn down and
     * the HAL runtime state is re-seeded for the new backend; for MKS the transport is created
     * later by the explicit CONNECT command (existing flow, nothing implicit).
     */
    Result<void, ErrorCode> setRealtimeInterfaceType(InterfaceConfig::RealtimeInterfaceType type);
    /** @brief Currently configured realtime backend type (startup value or last live switch). */
    [[nodiscard]] InterfaceConfig::RealtimeInterfaceType getRealtimeInterfaceType() const;

    /** @brief Applies runtime per-axis HAL configuration (limits, homing params) to the active driver. */
    Result<void, ErrorCode> setHalConfig(const HalConfigCommand& cmd);
    /**
     * @brief Forwards a homing request to HAL and lets HAL/runtime state report progress.
     * @param axis_id -1 for all axes, otherwise 0-based axis index.
     */
    Result<void, ErrorCode> requestHoming(int axis_id);
    /**
     * @brief Forwards a clear-faults request to the active driver's hardware.
     * @param axis_id -1 for all axes, otherwise 0-based axis index.
     */
    Result<void, ErrorCode> clearDriveErrors(int axis_id = -1);
    /** @brief Returns the current HAL runtime state snapshot (per-axis HAL config and status). */
    [[nodiscard]] HalConfigState getCurrentHalState() const;

private:
    // Returns a shared handle to the currently active driver. Callers must operate on the
    // returned shared_ptr copy (not a raw pointer) so a concurrent real_driver_ swap on the
    // controller thread cannot free a driver the RT loop is mid-call on (see C1).
    [[nodiscard]] std::shared_ptr<IDriver> getActiveDriverInstance();
    // Snapshot of the realtime driver pointer under real_driver_mutex_.
    [[nodiscard]] std::shared_ptr<IDriver> snapshotRealDriver() const;
    // Single routing rule for hardware service commands (homing, mastering, clear-faults,
    // E-Stop): prefer the real driver whenever it exists — even in Simulation view — and fall
    // back to the active (sim) driver only when no real driver is configured/connected.
    [[nodiscard]] std::shared_ptr<IDriver> serviceTargetDriver();
    // Detach and stop the realtime driver: swap the pointer out under the lock, then stop the
    // old instance with the lock released (stop() may block on the socket).
    void resetRealDriver();
    [[nodiscard]] AxisSet applyCommandGovernor(const AxisSet& target);
    void initializeHalState();
    void applyAxisCommandToState(size_t axis_index, const HalAxisConfigCommand& axis_cmd, HalConfigState& state) const;

    // --- setHalConfig() paths (one per HalTransportCommand) ---
    Result<void, ErrorCode> disconnectTransport(HalConfigState desired_state);
    Result<void, ErrorCode> connectTransport(uint32_t request_id, HalConfigState desired_state);
    Result<void, ErrorCode> applyConfigToDrivers(uint32_t request_id,
                                                 const HalConfigState& current_state,
                                                 HalConfigState desired_state);
    // Forwards a HAL config state to one driver with uniform success/failure logging.
    Result<void, ErrorCode> forwardHalConfig(IDriver& driver, const char* driver_name,
                                             const HalConfigState& state, uint32_t request_id);
    // Publishes the given state as the current HAL runtime state (under hal_config_mutex_).
    void commitHalState(const HalConfigState& state);

    // Owned copies: the HAL outlives any caller scope, so it must not reference caller-owned
    // configuration (config_ is also consulted at runtime by reinitializeRealtimeDriver()).
    const InterfaceConfig config_;
    const RobotLimits limits_;

    // LIVE realtime backend type — starts as config_.realtime_type and is the ONLY runtime-mutable
    // piece of the otherwise immutable startup configuration (HAL-panel backend dropdown, boss
    // directive 2026-07-06). Atomic: read on the RT path (read()/write() routing) while written
    // from the controller NRT thread via setRealtimeInterfaceType().
    std::atomic<InterfaceConfig::RealtimeInterfaceType> realtime_type_;

    // sim_driver_ is created once and never reassigned, so it needs no extra guard.
    // real_driver_ is reassigned/reset from the controller (NRT) thread while the RT loop
    // dereferences it every cycle, so every access goes through real_driver_mutex_ (C1).
    std::shared_ptr<IDriver> sim_driver_;
    std::shared_ptr<IDriver> real_driver_;
    mutable std::mutex real_driver_mutex_;
    std::string runtime_mks_tcp_ip_;
    uint16_t runtime_mks_tcp_port_{0};

    std::atomic<HalMode> active_mode_{HalMode::Simulation};

    // --- Command Governor state (RT write() path) ---
    // last_sent_cmd_/last_cmd_time_ are touched only inside applyCommandGovernor() on the RT
    // thread. is_first_cmd_ is atomic because NRT service paths (setMode, mastering, brakes)
    // reset it while the RT thread reads it.
    AxisSet last_sent_cmd_{};
    std::chrono::steady_clock::time_point last_cmd_time_;
    std::atomic<bool> is_first_cmd_{true};
    // RT-thread-only latch: the governor logs ONE warning per motion burst when it starts
    // clamping the stream (reset together with the governor on is_first_cmd_). Without it the
    // clamp is silent interference — the actual lags the planner and the resulting
    // Error_FollowingError is undiagnosable from the logs (found 2026-07-07).
    bool governor_clamp_logged_{false};

    // --- Synchronous Real Robot Keep-Alive ---
    // In Simulation mode the real robot is kept alive by sending Hold commands and reading its
    // status synchronously within the RT loop (no background thread by design). The latest real
    // feedback is cached here for the keep-alive Hold command and getRealDriverFeedback().
    HardwareFeedback real_feedback_{};
    mutable std::mutex real_feedback_mutex_;

    mutable std::shared_mutex hal_config_mutex_;
    HalConfigState hal_config_current_{};
};

} // namespace RDT
