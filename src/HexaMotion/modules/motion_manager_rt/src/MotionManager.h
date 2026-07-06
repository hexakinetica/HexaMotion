// MotionManager.h
#pragma once

#include "DataTypes.h"
#include "Units.h"
#include "TrajectoryQueue.h"
#include "LoggingMacros.h"
#include "RobotConfig.h" // For RobotLimits
#include "HardwareManager.h"

#include <thread>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <array>
#include <cstddef>

namespace RDT {

/**
 * @class RtPointRing
 * @brief Fixed-capacity FIFO for the RT loop's local command buffer (TrajectoryPoint cleanup
 *        Batch D).
 *
 * Measured motivation (2026-07-06): sizeof(TrajectoryPoint) is 800 bytes, larger than libstdc++'s
 * 512-byte deque block, so the previous std::deque buffer allocated AND freed one heap block on
 * EVERY steady-state RT cycle (measured 1.0 alloc/cycle over 10k cycles with a counting
 * allocator) — ~2000 heap operations per second on the RT thread. The ring's storage is a plain
 * member array (TrajectoryPoint is trivially copyable, NFR-DT-04), so the RT loop performs zero
 * heap traffic. RT-thread-only, like the deque it replaces: no locking, no atomics.
 */
class RtPointRing {
public:
    /// One-shot sizing: the refill loop tops up to MotionManager::RT_BUFFER_REFILL_THRESHOLD (25),
    /// so 32 gives headroom while staying a single memory page order. The ring never grows.
    static constexpr std::size_t kCapacity = 32;

    [[nodiscard]] bool empty() const noexcept { return count_ == 0; }
    [[nodiscard]] std::size_t size() const noexcept { return count_; }
    void clear() noexcept { head_ = 0; count_ = 0; }

    /// Precondition: !empty() — callers guard, matching the std::deque contract this replaces.
    [[nodiscard]] const TrajectoryPoint& front() const noexcept { return slots_[head_]; }
    void pop_front() noexcept {
        head_ = (head_ + 1) % kCapacity;
        --count_;
    }

    /// @return false when full: the caller must stop refilling — a motion point is NEVER dropped
    ///         silently (the refill invariant size() < RT_BUFFER_REFILL_THRESHOLD makes this
    ///         unreachable in production; the check is the fail-visible guard).
    [[nodiscard]] bool push_back(const TrajectoryPoint& point) noexcept {
        if (count_ == kCapacity) {
            return false;
        }
        slots_[(head_ + count_) % kCapacity] = point;
        ++count_;
        return true;
    }

private:
    std::array<TrajectoryPoint, kCapacity> slots_{};
    std::size_t head_ = 0;
    std::size_t count_ = 0;
};

/**
 * @class MotionManager
 * @brief The Real-Time (RT) heart of the motion control system.
 *
 * @details This class is responsible for the deterministic, high-frequency execution of motion commands.
 * Its primary roles are:
 * 1.  Pulling pre-calculated `TrajectoryPoint` commands from a lock-free queue filled by the non-real-time `TrajectoryPlanner`.
 * 2.  Sending the joint position targets from these points to the `HardwareManager` at a fixed rate (the RT-cycle tick).
 * 3.  Reading the latest feedback from the `HardwareManager`.
 * 4.  Performing critical real-time safety checks, such as Following Error monitoring.
 * 5.  Packaging the command, feedback, and diagnostic information into a `TrajectoryPoint` and pushing it to an outgoing queue for the `RobotController`.
 *
 * @note This class is designed to be lean and predictable. It does NOT perform complex calculations like trajectory generation,
 *       IK/FK, or velocity clamping. These tasks are handled by the `TrajectoryPlanner` and `HardwareManager` respectively.
 *
 * @version 2.0 (Refactored)
 */
class MotionManager {
public:
    /**
     * @brief Constructs the MotionManager.
     * @param hw_manager A shared pointer to the HAL facade (HardwareManager). Must not be null.
     * @param cycle_period_ms The period of the real-time execution loop in milliseconds.
     * @param limits The physical limits of the robot (used for position and following error checks).
     * @param following_error_threshold The maximum allowed deviation between commanded and actual joint positions before triggering an error.
     */
    MotionManager(std::shared_ptr<HardwareManager> hw_manager,
                  unsigned int cycle_period_ms,
                  const RobotLimits& limits,
                  Degrees following_error_threshold);

    ~MotionManager();

    // MotionManager owns a thread and manages hardware interaction, so it's non-copyable/movable.
    MotionManager(const MotionManager&) = delete;
    MotionManager& operator=(const MotionManager&) = delete;
    MotionManager(MotionManager&&) = delete;
    MotionManager& operator=(MotionManager&&) = delete;

    /**
     * @brief Starts the real-time execution thread.
     * @return `true` if the thread was started successfully, `false` otherwise (e.g., if HAL is not connected).
     */
    [[nodiscard]] bool start();

    /**
     * @brief Stops the real-time execution thread and waits for it to join.
     */
    void stop();

    /**
     * @brief Triggers an emergency stop.
     * This immediately clears all command queues and puts the manager into an error state.
     */
    void emergencyStop();

    /**
     * @brief Resets the manager from an error or stopped state.
     * Clears queues and resets the state to Idle, ready to accept new commands.
     */
    void reset();

    /**
     * @brief Enqueues a single trajectory point command from the NRT world.
     * This is the entry point for commands from the TrajectoryPlanner.
     * @param cmd_point The TrajectoryPoint to add to the command queue.
     * @return `true` if the point was enqueued, `false` if the queue was full.
     */
    [[nodiscard]] bool enqueueCommand(const TrajectoryPoint& cmd_point);

    /**
     * @brief Dequeues a feedback point for the NRT world to process.
     * This is the exit point for feedback to the RobotController.
     * @param[out] out_point The dequeued TrajectoryPoint.
     * @return `true` if a point was dequeued, `false` if the queue was empty.
     */
    [[nodiscard]] bool dequeueFeedback(TrajectoryPoint& out_point);

    /** @brief Gets the approximate size of the incoming command queue. */
    [[nodiscard]] size_t getCommandQueueSize() const;
    /** @brief Gets the approximate size of the outgoing feedback queue. */
    [[nodiscard]] size_t getFeedbackQueueSize() const;
    /** @brief Gets the current real-time state of the manager. */
    [[nodiscard]] RTState getCurrentState() const;

private:
    /** @internal @brief The main real-time loop function, executed in its own thread.
     *  Paces itself on an absolute next-cycle schedule with a 1 ms Windows timer resolution
     *  request for the thread lifetime (REQ_rt_cycle_pacing: the default ~15.6 ms sleep
     *  granularity made the 4 ms cycle — and therefore all streamed motion — run ~3.9x slower). */
    void rt_cycle_tick(std::stop_token stoken);

    /** @internal @brief Fills the internal RT buffer from the main NRT command queue if space is available. */
    void serviceMainCommandQueue(TrajectoryPoint& current_feedback_packet);

    /** @internal @brief Validates a new target point against joint position limits. */
    [[nodiscard]] bool validateTargetPoint(TrajectoryPoint& point);

    /** @internal @brief Checks the difference between the last command and current feedback. */
    [[nodiscard]] bool checkFollowingError(const AxisSet& last_cmd, const AxisSet& current_fb, TrajectoryPoint& out_point);
    [[nodiscard]] bool isHomingActive() const;
    
    std::shared_ptr<HardwareManager> hw_manager_;
    
    const std::chrono::milliseconds cycle_period_;
    const RobotLimits limits_;
    const Degrees following_error_threshold_;
    std::jthread rt_thread_;

    std::atomic<bool> running_{false};
    std::atomic<bool> reset_request_{false};
    std::atomic<bool> reset_ack_{false};
    std::atomic<bool> estop_request_{false};
    std::atomic<RTState> current_state_{RTState::Idle};
    std::atomic<bool> hal_motion_active_{false};

    bool dependencies_valid_{true};
    
    TrajectoryQueue<TrajectoryPoint, 512> command_queue_;

    // Outgoing feedback is a latest-value mailbox, NOT a FIFO. The NRT consumer only needs the most
    // recent robot state, so a FIFO that overflows would keep stale samples and drop the freshest one
    // (the opposite of what we want). The RT producer overwrites the latest sample (drop-oldest by
    // construction); the consumer takes it at most once. Guarded by a short mutex around a single
    // struct copy — negligible against the 4 ms RT budget — which also removes the previous SPSC
    // violation (the RT producer used to call try_pop() on the feedback queue).
    mutable std::mutex feedback_mutex_;
    TrajectoryPoint feedback_value_{};
    bool feedback_fresh_{false};

    // While in Error state the RT loop skips processing, so the diagnostic that caused the error
    // (e.g. Error_JointLimit) would otherwise appear in only one feedback packet. Latch it and
    // re-publish it every cycle until reset, so the latest-value mailbox cannot drop the error
    // reason (and the HMI shows it persistently). Cleared on reset. RT-thread-only.
    bool error_latched_{false};
    PipelineDiagnostics latched_diagnostics_{};

    // Small, fast internal buffer for the RT loop to consume from. Fixed ring (Batch D): the
    // previous std::deque allocated one heap block per RT cycle in steady state (see RtPointRing).
    RtPointRing rt_command_buffer_;
    TrajectoryPoint segment_end_hold_point_{};
    bool waiting_for_segment_completion_{false};
    AxisSet last_sent_joints_{};
    
    // Threshold to trigger refilling the RT buffer from the main queue.
    static constexpr size_t RT_BUFFER_REFILL_THRESHOLD = 25;

    static inline const std::string MODULE_NAME = "MotionManager";
};

} // namespace RDT
