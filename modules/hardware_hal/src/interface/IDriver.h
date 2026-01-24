// IDriver.h
#ifndef I_DRIVER_H
#define I_DRIVER_H

#pragma once

#include "../HalTypes.h"
#include "result.h"
#include "ErrorCode.h"

namespace RDT {

/**
 * @class IDriver
 * @brief Internal abstraction for hardware backends.
 *
 * Implements REQ-HAL-02, REQ-HAL-NFR-02.
 * Implementations (Sim, UDP, EtherCAT) must ensure thread safety for
 * read/write operations called from the RT loop.
 *
 * @section Offsets
 * Drivers are responsible for converting between Raw (Physical) coordinates
 * and Logical coordinates used by the controller via an internal Offset map.
 */
class IDriver {
public:
    virtual ~IDriver() = default;

    /**
     * @brief Initialize connection/resources.
     */
    [[nodiscard]] virtual Result<void, ErrorCode> init() = 0;

    /**
     * @brief Close connection and stop background threads.
     */
    virtual void stop() = 0;

    /**
     * @brief Send positions to hardware.
     * @param cmd Target LOGICAL joint positions. Driver must convert to Physical.
     * Must be non-blocking.
     */
    [[nodiscard]] virtual Result<void, ErrorCode> write(const AxisSet& cmd) = 0;

    /**
     * @brief Read latest feedback (cached).
     * @return HardwareFeedback with LOGICAL positions (Physical + Offset).
     * Must be wait-free/lock-free.
     */
    [[nodiscard]] virtual Result<HardwareFeedback, ErrorCode> read() = 0;

    /**
     * @brief Recalculates the internal offset for an axis such that the current physical position
     * corresponds to the given logical 'new_zero_deg'.
     *
     * Logic: NewOffset = TargetLogical - CurrentPhysical
     *
     * @param axis The axis to zero/calibrate.
     * @param new_zero_deg The desired logical value for the current physical position (usually 0.0).
     */
    [[nodiscard]] virtual Result<void, ErrorCode> resetAxisToZero(AxisId axis, double new_zero_deg) = 0;

    /**
     * @brief Forcibly sets the internal LOGICAL state of the driver.
     * Critical for Syncing Simulation to Real hardware.
     */
    virtual Result<void, ErrorCode> setState(const AxisSet& state) {
        (void)state;
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidArgument);
    }
};

} // namespace RDT

#endif // I_DRIVER_H
