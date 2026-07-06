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
#include "ProgramSequencer.h"

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
 * - Responding to network commands (program control, jogging, mode switching, calibration).
 * - Managing the program execution state machine.
 * - Initiating motion tasks by sending targets to the TrajectoryPlanner.
 * - Monitoring feedback from the MotionManager.
 *
 * @note This class does NOT have its own execution thread. Its `update()` method
 *       must be called periodically from the main application loop.
 *
 * @version 2.5 (REQ_program_pause_resume: RESUME re-plans the interrupted chain remainder;
 *               PAUSE/STOP hold at the physical position; line tracking live in WaitingMotion)
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
                    std::shared_ptr<RobotState> robot_state,
                    const ControllerConfig& config);
    
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
    
    /** @brief Checks if the entire planned trajectory has been sent to the MotionManager and executed. */
    [[nodiscard]] bool isTaskFinished() const;

private:
    // --- Internal Logic ---
    void processNetworkCommands();
    void processMotionFeedback();
    void handleProgramExecution();
    void syncHalRuntimeState();
    
    // --- Command Handlers ---
    void handleModeSwitch(bool enable_real_mode);
    void handleJogRequest(const NetProtocol::ControlState& cmd);
    void handleHalDirectJog(const NetProtocol::ControlState& cmd);
    void handleProgramCommand(int command);
    void handleEmergencyStop(bool active);
    void handleClearError();
    void handleJogEnableCommand(bool enabled);
    void handleMasteringRequest(const NetProtocol::ControlState& cmd);
    void handleHomingRequest(const NetProtocol::ControlState& cmd);
    // Applies a changed realtime-backend selection ("sim"/"udp"/"mks_tcp") to the HAL, with an
    // operator-visible system message on both success and refusal (fail-closed in REAL mode).
    void handleRealtimeBackendSelection(const std::string& backend);
    void handleProgramUpdateRequest(const NetProtocol::ControlState& cmd);

    // --- Program Execution Helpers ---
    // The flat-program interpretation lives in ProgramSequencer (loops/branches/waits, fail-closed
    // validation, runaway watchdog). RobotController owns the timers/planner/HAL and ACTUATES the
    // sequencer's decisions, so the command -> hardware data flow stays visible here.
    void startProgram();
    void stopProgram();
    void pauseProgram();
    /// Ask the sequencer for the next action (with the latest DI snapshot) and actuate it.
    void driveSequencer();
    /// Perform one StepAction: plan a motion chain, start a time/input wait, or finish the program.
    void dispatchAction(const StepAction& action);
    /// Convert program Move steps into planner waypoints (lines tagged from @p first_line) and plan
    /// them as one continuous chain. Shared by dispatchAction and the pause/resume re-plan
    /// (REQ-PAUSE-02). On a planner error it faults the program and returns false.
    [[nodiscard]] bool planChainSteps(const std::vector<NetProtocol::ProgramStepStruct>& steps,
                                      std::int32_t first_line);
    /// Latest feedback point with the command retargeted to the PHYSICAL joints (joint_actual),
    /// FK-enriched (fail-closed cartesian_valid=false if FK fails). PAUSE/STOP hold the robot at
    /// this point so firmware-profiling backends stop where the robot IS (REQ-PAUSE-01).
    [[nodiscard]] TrajectoryPoint physicalHoldPoint() const;
    /// Latch a program fault: stop, report a typed diagnostic to the operator, and enter Error mode.
    void faultProgram(const std::string& reason);

    // --- Kinematics helpers ---
    [[nodiscard]] bool solveFK(const AxisSet& joints, CartPose& world_pose) const;
    [[nodiscard]] Result<AxisSet, IKError> solveIK(const CartPose& world_pose,
                                                                const AxisSet& seed_joints) const;
    
    /** @brief Validates if the robot is ready to move (no errors, enabled, etc.). */
    [[nodiscard]] bool isRobotReadyForMotion() const;



    // --- Dependencies ---
    std::shared_ptr<RobotState> robot_state_;
    std::shared_ptr<HardwareManager> hw_manager_;
    std::shared_ptr<MotionManager> motion_manager_;
    std::shared_ptr<TrajectoryPlanner> planner_;
    std::shared_ptr<KinematicSolver> solver_;

    // --- Configuration ---
    const ControllerConfig& config_;

    // --- Internal State ---
    // Controller-side execution phase. The sequencer decides WHAT to do next; these states track the
    // controller's servicing of a blocking action (motion in flight / timed wait / input wait) and own
    // the wall-clock timers the sequencer deliberately does not.
    enum class ExecutionState { Stopped, Running, Paused, WaitingMotion, WaitingTime, WaitingDI };
    ExecutionState exec_state_{ExecutionState::Stopped};

    // The flat-program interpreter (loops/branches/waits). Loaded when a program arrives, re-armed on
    // a fresh RUN, driven every tick while running.
    ProgramSequencer sequencer_;

    std::chrono::steady_clock::time_point wait_start_time_;
    Seconds wait_duration_{0.0};   ///< Active WaitTime duration.
    Seconds wait_timeout_{0.0};    ///< Active WaitDI timeout (0 = wait indefinitely).

    // PAUSE freezes an active wait: remember which blocking phase we paused from so RESUME restores it
    // (and restarts a frozen wait from its remaining time) instead of re-deciding into a stuck Idle.
    ExecutionState state_before_pause_{ExecutionState::Running};

    // The program-step line of the WaitDI we are currently blocked on. Used to start the timeout only
    // when a NEW WaitDI begins (line changes), not on every re-poll, so the timeout stays accurate
    // across re-polls and across loop iterations. -1 = not waiting on any input.
    std::int32_t waiting_di_line_{-1};

    // --- Interrupted-chain resume context (REQ_program_pause_resume) ---
    // A motion run is dispatched as ONE chain and the sequencer's program counter is already past
    // it, so PAUSE/RESUME must re-plan the chain's remaining waypoints itself: the dispatched steps,
    // the program line of the first one, and the waypoint line the robot is physically executing
    // (tracked from feedback sequence_index). Cleared on STOP, chain completion and fresh RUN.
    std::vector<NetProtocol::ProgramStepStruct> active_motion_chain_;
    std::int32_t active_chain_first_line_{-1};
    std::int32_t executing_motion_line_{-1};

    // Tracking network command changes to ensure idempotency
    uint32_t last_processed_prog_update_id_{0};
    uint32_t last_program_cmd_req_id_{0};   // audit F2: RUN/PAUSE/STOP now deduped by programCmdReqId
    uint32_t last_mastering_req_id_{0};
    uint32_t last_homing_req_id_{0};
    uint32_t last_mode_req_id_{0};
    int last_jog_enable_cmd_{-1};
    uint32_t last_hal_config_req_id_{0};
    // Last realtime-backend selection ("sim"/"udp"/"mks_tcp") applied to the HAL. Initialised from
    // the persisted config in initialize(), so the startup value never triggers a live switch; a
    // changed value (HAL panel dropdown, riding the config transaction) is applied exactly once.
    std::string last_applied_realtime_backend_;

    // Audit F2: safety-command actuation is edge-detected here so it fires exactly once per
    // transition, driven by the reliably-latched RobotState level (E-Stop) or a rising edge on the
    // one-shot flag (Clear Error), instead of the racy transient wire flag polled every NRT cycle.
    bool estop_actuated_{false};        ///< True while the E-Stop hardware actuation is engaged.
    bool clear_error_actuated_{false};  ///< Rising-edge latch for the one-shot Clear Error flag.

    bool dependencies_valid_{true};

    // Tracking for Ghost entity fallback
    AxisSet last_valid_real_joints_{};

    // --- KUKA-style Cartesian jog state ---
    // The Cartesian jog accumulates increments on a maintained TCP target and IK seed instead of
    // re-deriving them from the (lagging) actual joints on every click. This keeps orientation and
    // position increments clean and continuous, and lets the jog rotate ABOUT THE TCP (so the TCP
    // stays fixed during an orientation jog, like on a KUKA). The target is re-synced from the live
    // robot when a fresh jog starts (re-arm) or when it has drifted (an external move).
    bool jog_active_{false};
    CartPose jog_tcp_world_{};   ///< Maintained TCP pose in world (the Cartesian jog target).
    AxisSet jog_seed_joints_{};  ///< IK seed / last committed jog joints (configuration continuity).

    static inline const std::string MODULE_NAME = "RobotCtrl";
};

} // namespace RDT