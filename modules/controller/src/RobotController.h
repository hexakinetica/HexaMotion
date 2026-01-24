// RobotController.h
#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#pragma once

#include "DataTypes.h"
#include "Units.h"
#include "LoggingMacros.h"
#include "RobotState.h"
#include "HardwareManager.h"
#include "MotionManager.h"
#include "planner/TrajectoryPlanner.h"
#include "kinematic_solver/KinematicSolver.h"
#include "RobotConfig.h"

#include <memory>
#include <string>
#include <atomic>
#include <vector>
#include <chrono>

namespace RDT {

/**
 * @class RobotController
 * @brief The central orchestrator of the robot's logic.
 *
 * @details This class is the brain of the robot controller. It connects the high-level
 * state and commands (from RobotState) to the motion execution pipeline.
 *
 * Responsibilities:
 * - Responding to network commands (program control, jogging, mode switching).
 * - Managing the program execution state machine.
 * - Initiating motion tasks by sending targets to the TrajectoryPlanner.
 * - Monitoring feedback from the MotionManager.
 *
 * @note This class does NOT have its own execution thread. Its `update()` method
 *       must be called periodically from the main application loop.
 *
 * @version 2.0 (Refactored)
 */
class RobotController {
public:
    /**
     * @brief Constructs the RobotController with injected dependencies.
     */
    RobotController(std::shared_ptr<HardwareManager> hw_manager,
                    std::shared_ptr<MotionManager> motion_manager,
                    std::shared_ptr<TrajectoryPlanner> planner,
                    std::shared_ptr<KinematicSolver> solver,
                    std::shared_ptr<RobotState> robot_state);
    
    ~RobotController();

    RobotController(const RobotController&) = delete;
    RobotController& operator=(const RobotController&) = delete;

    /**
     * @brief Initializes the controller and aligns internal state with hardware.
     * @return `true` on successful initialization.
     */
    [[nodiscard]] bool initialize();

    /**
     * @brief Main processing tick. Called from main.cpp.
     */
    void update();

private:
    // --- Internal Logic ---
    void processNetworkCommands();
    void processMotionFeedback();
    void handleProgramExecution();
    
    // --- Command Handlers ---
    void handleModeSwitch(bool enable_real_mode);
    void handleJogRequest(const NetProtocol::ControlState& cmd);
    void handleProgramCommand(int command);
    void handleEmergencyStop(bool active);
    void handleClearError();

    // --- Program Execution Helpers ---
    void startProgram();
    void stopProgram();
    void pauseProgram();
    void executeNextStep();
    void handleWaitStep(const NetProtocol::ProgramStepStruct& step);
    
    /** @brief Validates if the robot is ready to move (no errors, enabled, etc.). */
    [[nodiscard]] bool isRobotReadyForMotion() const;

    // --- Dependencies ---
    std::shared_ptr<RobotState> robot_state_;
    std::shared_ptr<HardwareManager> hw_manager_;
    std::shared_ptr<MotionManager> motion_manager_;
    std::shared_ptr<TrajectoryPlanner> planner_;
    std::shared_ptr<KinematicSolver> solver_;

    // --- Internal State ---
    // State machine for program execution
    enum class ExecutionState { Stopped, Running, Paused, WaitingTime, WaitingDI };
    ExecutionState exec_state_{ExecutionState::Stopped};
    
    size_t current_program_step_index_{0};
    std::chrono::steady_clock::time_point wait_start_time_;
    Seconds wait_duration_{0.0};

    // Tracking network command changes
    uint32_t last_processed_jog_id_{0};
    uint32_t last_processed_config_req_id_{0};
    uint32_t last_processed_prog_req_id_{0};
    int last_program_cmd_{0};
    bool last_real_mode_req_{false};
    bool last_estop_req_{false};

    static inline const std::string MODULE_NAME = "RobotCtrl";
};

} // namespace RDT
#endif // ROBOTCONTROLLER_H
