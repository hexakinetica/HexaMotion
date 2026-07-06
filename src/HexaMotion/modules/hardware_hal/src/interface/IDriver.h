// IDriver.h
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
     * @brief Send a full runtime HAL motion command.
     * @param command Stream point plus optional final segment target metadata.
     * Drivers that do not use the segment target may fall back to `write(stream_target)`.
     */
    [[nodiscard]] virtual Result<void, ErrorCode> writeCommand(const HardwareCommand& command) {
        return write(command.stream_target);
    }

    /**
     * @brief Drive one digital output (program SET DO step, sequencer P3).
     * @param port 1-based DO port; bit = port-1 in the 32-bit output word.
     * @param state Level to drive.
     * Default: REFUSED with a typed error. A backend without a digital-output channel must not
     * silently "accept" the write - the running program faults explicitly instead of pretending an
     * output was set (Simplicity Mandate: no silent failures).
     */
    [[nodiscard]] virtual Result<void, ErrorCode> setDigitalOutput(uint16_t port, bool state) {
        (void)port;
        (void)state;
        return Result<void, ErrorCode>::Failure(ErrorCode::NotSupported);
    }

    /**
     * @brief Read latest feedback (cached).
     * @return HardwareFeedback with LOGICAL positions (Physical + Offset).
     * Must be wait-free/lock-free.
     */
    [[nodiscard]] virtual Result<HardwareFeedback, ErrorCode> read() = 0;

    /**
     * @brief Recalculates the internal offset for an axis such that the current physical position
     * corresponds to the given logical position.
     *
     * Logic: NewOffset = TargetLogical - CurrentPhysical
     *
     * @param axis The axis to zero/calibrate.
     * @param logical_position The desired logical value for the current physical position.
     */
    [[nodiscard]] virtual Result<void, ErrorCode> masterAxisAt(AxisId axis, Degrees logical_position) = 0;

    /**
     * @brief Zero ALL axes to logical 0.0. Default loops masterAxisAt() per axis; backends with a
     * native bulk command (e.g. the MKS SetZeroAll) override this to send a single atomic request.
     */
    [[nodiscard]] virtual Result<void, ErrorCode> masterAllAxes() {
        for (std::size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            auto res = masterAxisAt(static_cast<AxisId>(i), Degrees(0.0));
            if (res.isError()) return res;
        }
        return Result<void, ErrorCode>::Success();
    }

    /**
     * @brief Best-effort emergency stop of all axes on the backend.
     *
     * Default is a no-op Success: drivers that execute the per-cycle stream (sim/UDP) are
     * halted by MotionManager::emergencyStop() freezing the stream. Backends that profile
     * motion in firmware (e.g. the MKS Motor Configurator runs its own trapezoid to the
     * commanded endpoint) MUST override this to actively stop the drives, otherwise an
     * in-flight move completes after the operator's E-Stop (audit F-13).
     *
     * Ownership note: implementations send ordinary service commands subject to the
     * backend's ownership gate — by project decision nothing bypasses that gate, so with
     * owner=UI the backend rejects these and its own local E-Stop is the physical stop.
     */
    [[nodiscard]] virtual Result<void, ErrorCode> emergencyStopAll() {
        return Result<void, ErrorCode>::Success();
    }

    /**
     * @brief Forcibly sets the internal LOGICAL state of the driver.
     * Critical for Syncing Simulation to Real hardware.
     */
    virtual Result<void, ErrorCode> setState(const AxisSet& state) {
        (void)state;
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidArgument);
    }

    /**
     * @brief Engages or disengages the brakes.
     * @param engaged True to engage the brakes, false to disengage.
     */
    [[nodiscard]] virtual Result<void, ErrorCode> setBrakeState(bool engaged) = 0;

    /**
     * @brief Applies runtime HAL configuration/state to a backend if supported.
     * Useful for propagating per-axis HAL state to an external UDP HAL peer/backend.
     */
    virtual Result<void, ErrorCode> setHalConfig(const HalConfigState& hal_state) {
        (void)hal_state;
        return Result<void, ErrorCode>::Success();
    }

    /**
     * @brief Requests native HAL homing execution for one axis or all axes.
     * @param axis_id -1 for all axes, otherwise 0-based axis index.
     */
    virtual Result<void, ErrorCode> requestHoming(int axis_id) {
        (void)axis_id;
        return Result<void, ErrorCode>::Failure(ErrorCode::InvalidArgument);
    }

    /**
     * @brief Clears latched drive faults/protection on the hardware.
     * @param axis_id -1 for all axes, otherwise 0-based axis index.
     * Default no-op for backends without a clearable fault state (e.g. SimDriver).
     */
    virtual Result<void, ErrorCode> clearDriveErrors(int axis_id = -1) {
        (void)axis_id;
        return Result<void, ErrorCode>::Success();
    }

    /**
     * @brief Returns the driver's current HAL runtime state snapshot.
     */
    [[nodiscard]] virtual HalConfigState getCurrentHalState() const {
        return HalConfigState{};
    }

    /**
     * @brief Returns only the per-axis runtime state (trivially copyable slice).
     *
     * The RT loop refreshes its per-axis mirror every cycle. The full HalConfigState carries
     * std::string diagnostics (transport_ip, last_ipc_error, homing texts) whose copy would
     * heap-allocate on the RT path, so the hot path uses this POD slice instead. The default
     * derives from getCurrentHalState(); drivers holding the state under a mutex override it
     * to copy the axes array directly.
     */
    [[nodiscard]] virtual std::array<HalAxisConfigState, ROBOT_AXES_COUNT> getAxisRuntimeState() const {
        return getCurrentHalState().axes;
    }
};

} // namespace RDT
