// SimDriver.h
#pragma once

#include "../../interface/IDriver.h"
#include "../AxisOffsets.h"
#include <atomic>
#include <array>
#include <mutex>

namespace RDT {

/**
 * @class SimDriver
 * @brief Internal simulation backend.
 * Holds state in memory and supports Offset Management for testing calibration flows.
 */
class SimDriver : public IDriver {
public:
    /**
     * @brief Simulated drive over the given robot limits.
     * The HAL velocity/soft limits are seeded from the ROBOT limits (not an arbitrary constant):
     * the command governor clamps the REAL-view stream to the HAL velocity limit, so a default
     * below the planner's allowed speed makes the actual lag the command and trips
     * Error_FollowingError on fast moves (found with a 100 deg/s hardcode, 2026-07-07).
     */
    SimDriver(const AxisSet& initial_state, const RobotLimits& limits);
    ~SimDriver() override = default;

    // NOTE: writeCommand is deliberately NOT overridden. MotionManager tags EVERY streamed point
    // with segment metadata (for the MKS firmware), so a segment-consuming SimDriver would
    // teleport the robot to the segment endpoint mid-move and trip Error_FollowingError on long
    // moves (100 mm regression, 2026-07-07). SimDriver stays a pure stream follower; the virtual
    // MC jog is applied via setState (HardwareManager::jogRealIncremental, REQ-SIMMC-05).
    Result<void, ErrorCode> init() override;
    void stop() override;
    Result<void, ErrorCode> write(const AxisSet& cmd) override;
    Result<HardwareFeedback, ErrorCode> read() override;
    
    // --- Calibration ---
    Result<void, ErrorCode> masterAxisAt(AxisId axis, Degrees logical_position) override;

    // --- Sync Logic ---
    Result<void, ErrorCode> setState(const AxisSet& state) override;

    [[nodiscard]] Result<void, ErrorCode> setBrakeState(bool engaged) override;
    [[nodiscard]] Result<void, ErrorCode> setDigitalOutput(uint16_t port, bool state) override;
    Result<void, ErrorCode> setHalConfig(const HalConfigState& hal_state) override;
    Result<void, ErrorCode> requestHoming(int axis_id) override;
    [[nodiscard]] HalConfigState getCurrentHalState() const override;
    [[nodiscard]] std::array<HalAxisConfigState, ROBOT_AXES_COUNT> getAxisRuntimeState() const override;

private:
    // We store "Physical" state to properly simulate the offset logic.
    std::atomic<AxisSet> physical_state_;

    // To simulate 1-cycle latency (ideal drive), we store what we will report next.
    std::atomic<AxisSet> next_report_state_;
    
    // Offsets: Logical = Physical + Offset (shared helper, audit B3).
    AxisOffsets offsets_;

    std::atomic<bool> m_brakesEngaged{true};

    // Digital outputs driven by the program (SET DO). SIM LOOPBACK: read() mirrors these bits into
    // feedback.digital_inputs, so SET DO + WAIT DI/IF programs can be dry-run without hardware
    // (deliberate, documented simulation behaviour - the real backends have independent DI).
    std::atomic<uint32_t> digital_outputs_{0};
    mutable std::mutex hal_state_mutex_;
    HalConfigState hal_state_{};
};

} // namespace RDT
