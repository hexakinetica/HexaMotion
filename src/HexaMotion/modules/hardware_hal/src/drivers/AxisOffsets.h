// AxisOffsets.h
#pragma once

#include "DataTypes.h" // for ROBOT_AXES_COUNT

#include <array>
#include <atomic>
#include <cstddef>

namespace RDT {

/**
 * @class AxisOffsets
 * @brief Per-axis logical/physical offset store shared by the HAL drivers (audit B3, Variant 3).
 *
 * All drivers convert between the controller's LOGICAL joint frame and the hardware's PHYSICAL frame
 * with the same convention, previously duplicated inline in each driver:
 *   - logical  = physical + offset
 *   - physical = logical  - offset
 *
 * This value type owns the offsets and the two conversions so the convention lives in one place. It
 * is held by composition (not inherited): the drivers differ in transport, threading and mastering,
 * so a shared base class is not justified. Mastering (how a new offset is computed) stays in each
 * driver because it genuinely differs (Sim/Udp compute offset = logical - physical; Mks sets
 * offset = logical after a hardware SetZero) — this class only stores and applies offsets.
 *
 * Thread-safety: the offsets are relaxed atomics, matching the previous per-driver members. The RT
 * read/write path and the NRT mastering path access them without a lock; each axis is an independent
 * atomic, so a single access can never tear. Ordering across axes is not required (each axis offset
 * is used independently), hence relaxed.
 */
class AxisOffsets {
public:
    AxisOffsets() {
        for (auto& offset : offsets_) {
            offset.store(0.0, std::memory_order_relaxed);
        }
    }

    /// @brief Converts a physical (raw hardware) angle to the controller's logical angle.
    [[nodiscard]] double toLogical(std::size_t axis, double physical_deg) const {
        return physical_deg + offsets_[axis].load(std::memory_order_relaxed);
    }

    /// @brief Converts a controller logical angle to the physical angle sent to the hardware.
    [[nodiscard]] double toPhysical(std::size_t axis, double logical_deg) const {
        return logical_deg - offsets_[axis].load(std::memory_order_relaxed);
    }

    /// @brief Reads the current offset for an axis (used by mastering to recompute a new offset).
    [[nodiscard]] double get(std::size_t axis) const {
        return offsets_[axis].load(std::memory_order_relaxed);
    }

    /// @brief Sets the offset for an axis (mastering result).
    void set(std::size_t axis, double offset_deg) {
        offsets_[axis].store(offset_deg, std::memory_order_relaxed);
    }

    /// @brief Resets an axis offset to zero (mastering to logical 0.0).
    void reset(std::size_t axis) {
        offsets_[axis].store(0.0, std::memory_order_relaxed);
    }

private:
    std::array<std::atomic<double>, ROBOT_AXES_COUNT> offsets_;
};

} // namespace RDT
