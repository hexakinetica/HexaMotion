// MotionManager.h
#ifndef MOTION_MANAGER_H
#define MOTION_MANAGER_H

#pragma once

#include "DataTypes.h"
#include "Units.h"
#include "TrajectoryQueue.h"
#include "LoggingMacros.h"
#include "RobotConfig.h" // For RobotLimits
#include "HardwareManager.h" // ЗАВИСИМОСТЬ ОТ НОВОГО HAL

#include <thread>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <deque>
#include <array>

namespace RDT {

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
     * @param hw_manager A shared pointer to the initialized HardwareManager. Must not be null.
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
    /** @internal @brief The main real-time loop function, executed in its own thread. */
    void rt_cycle_tick(std::stop_token stoken);

    /** @internal @brief Sleeps for the remainder of the cycle period. */
    void sleepUntilNextCycle(const std::chrono::steady_clock::time_point& cycle_start_time) const;
    
    /** @internal @brief Fills the internal RT buffer from the main NRT command queue if space is available. */
    void serviceMainCommandQueue(TrajectoryPoint& current_feedback_packet);

    /** @internal @brief Validates a new target point against joint position limits. */
    [[nodiscard]] bool validateTargetPoint(TrajectoryPoint& point);

    /** @internal @brief Checks the difference between the last command and current feedback. */
    [[nodiscard]] bool checkFollowingError(const AxisSet& last_cmd, const AxisSet& current_fb, TrajectoryPoint& out_point);
    
    std::shared_ptr<HardwareManager> hw_manager_; 
    
    const std::chrono::milliseconds cycle_period_;
    const RobotLimits limits_;
    const Degrees following_error_threshold_;
    std::jthread rt_thread_;

    std::atomic<bool> running_{false};
    std::atomic<RTState> current_state_{RTState::Idle};
    
    TrajectoryQueue<TrajectoryPoint, 512> command_queue_;
    TrajectoryQueue<TrajectoryPoint, 512> feedback_queue_;

    // Small, fast internal buffer for the RT loop to consume from.
    std::deque<TrajectoryPoint> rt_command_buffer_; 
    AxisSet last_sent_joints_{};
    
    // Threshold to trigger refilling the RT buffer from the main queue.
    static constexpr size_t RT_BUFFER_REFILL_THRESHOLD = 25;

    static inline const std::string MODULE_NAME = "MotionManager";
};

} // namespace RDT
#endif // MOTION_MANAGER_H
