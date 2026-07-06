// HalTypes.h
#pragma once

#include "DataTypes.h" // AxisSet, HalStatus, HalConfigState, etc.
#include <cstdint>
#include <array>

namespace RDT {

enum class HalCommandExecutionMode : uint8_t {
    Teleport = 0,
    InternalInterpolator = 1,
};

/**
 * @struct HardwareCommand
 * @brief Runtime HAL command packet passed from MotionManager to a driver.
 *
 * `stream_target` is the per-cycle command point.
 * `segment_target` is the final joint target of the currently active segment.
 * Drivers that do not need the segment target may ignore it.
 */
struct HardwareCommand {
    AxisSet stream_target{};
    TargetJointPose segment_target{};
    bool has_segment_target = false;

    [[nodiscard]] auto operator<=>(const HardwareCommand&) const = default;
};

/**
 * @struct SafetyState
 * @brief Monitor-only status of the hardware safety chain.
 *
 * Implements REQ-HAL-10, REQ-HAL-11.
 * The actual safety stop (STO) is hardware-wired. This struct is for
 * the software to react (display error, stop planner).
 */
struct SafetyState {
    bool is_estop_active = false;      ///< E-Stop chain is open (Button pressed).
    bool is_hard_limit_hit = false;    ///< Physical limit switch triggered.
    bool is_power_on = false;          ///< High voltage is present on drives.

    [[nodiscard]] auto operator<=>(const SafetyState&) const = default;
};

/**
 * @struct HardwareFeedback
 * @brief The complete status packet returned from the HAL cycle.
 *
 * Implements REQ-HAL-08, REQ-HAL-13.
 */
struct HardwareFeedback {
    AxisSet joints{};                ///< Current LOGICAL positions (Physical + Offset).
    uint32_t digital_inputs = 0;     ///< Raw DI bitmask (for Homing sensors).
    SafetyState safety{};            ///< Safety chain status.
    HalStatus driver_status = HalStatus::NotConnected; ///< Diagnostic health.
    HalCommandExecutionMode execution_mode = HalCommandExecutionMode::Teleport;
    bool target_reached = true;
    bool motion_active = false;

    /// @brief Active motion owner reported by the backend (0 = UI/local, 1 = HexaMotion).
    /// When this is not 1, the MKS Motor Configurator silently rejects every motion/
    /// service command issued by HexaMotion (the operator must hand control to
    /// HexaMotion in the configurator GUI). Surfaced so the UI can explain why the
    /// robot is connected but not moving (BUG-02/BUG-18).
    uint8_t control_owner_id = 1;

    [[nodiscard]] auto operator<=>(const HardwareFeedback&) const = default;
};

} // namespace RDT