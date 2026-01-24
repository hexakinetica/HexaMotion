// HalTypes.h
#ifndef HAL_TYPES_H
#define HAL_TYPES_H

#pragma once

#include "DataTypes.h" // AxisSet, HalStatus, Result, ErrorCode
#include <cstdint>

namespace RDT {

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

    [[nodiscard]] auto operator<=>(const HardwareFeedback&) const = default;
};

/**
 * @enum HalMode
 * @brief Operating mode of the HardwareManager.
 */
enum class HalMode {
    Simulation, ///< Internal math model (SimDriver).
    Realtime    ///< Physical hardware (Udp/EthercatDriver).
};

} // namespace RDT

#endif // HAL_TYPES_H