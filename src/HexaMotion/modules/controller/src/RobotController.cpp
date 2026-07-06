// RobotController.cpp
#include "RobotController.h"
#include "frame_processor/FrameTransformer.h"
#include <kdl/frames.hpp>
#include <cmath>
#include <algorithm>

namespace RDT {

using namespace RDT::literals;


bool RobotController::solveFK(const AxisSet& joints, CartPose& world_pose) const {
    if (!solver_) return false;
    return solver_->solveFK(joints, world_pose);
}

Result<AxisSet, IKError> RobotController::solveIK(const CartPose& world_pose,
                                                  const AxisSet& seed_joints) const {
    if (!solver_) {
        return Result<AxisSet, IKError>::Failure(IKError::InternalError);
    }
    return solver_->solveIK(world_pose, seed_joints);
}

RobotController::RobotController(std::shared_ptr<HardwareManager> hw_manager,
                                 std::shared_ptr<MotionManager> motion_manager,
                                 std::shared_ptr<TrajectoryPlanner> planner,
                                 std::shared_ptr<KinematicSolver> solver,
                                 std::shared_ptr<RobotState> robot_state,
                                 const ControllerConfig& config)
    : hw_manager_(hw_manager),
      motion_manager_(motion_manager),
      planner_(planner),
      solver_(solver),
      robot_state_(robot_state),
      config_(config)
{
    if (!hw_manager_ || !motion_manager_ || !planner_ || !solver_ || !robot_state_) {
        RDT_LOG_CRITICAL(MODULE_NAME, "Dependencies cannot be null.");
        dependencies_valid_ = false;
    }
}

RobotController::~RobotController() = default;

bool RobotController::initialize() {
    if (!dependencies_valid_) {
        robot_state_->updateRobotMode(RobotMode::Error);
        robot_state_->updateSystemMessage("Initialization failed: missing dependencies", true);
        return false;
    }
    RDT_LOG_INFO(MODULE_NAME, "Initializing RobotController...");

    // 1. Initialize Hardware
    auto hw_res = hw_manager_->init();
    if (hw_res.isError()) {
        RDT_LOG_CRITICAL(MODULE_NAME, "Hardware init failed: {}", ToString(hw_res.error()));
        robot_state_->updateRobotMode(RobotMode::Error);
        return false;
    }

    // 2. Read initial state from Hardware to sync planner
    auto fb_res = hw_manager_->read();
    if (fb_res.isError()) {
        RDT_LOG_CRITICAL(MODULE_NAME, "Failed to read initial state from HAL.");
        return false;
    }

    // 3. Initialize Planner State
    TrajectoryPoint initial_tp;
    initial_tp.command.joint_target = fb_res.value().joints;
    // Solve FK to get Cartesian pose
    if (solveFK(initial_tp.command.joint_target, initial_tp.command.cartesian_target)) {
        // FK successful, update the robot state with the calculated cartesian pose
        initial_tp.command.cartesian_valid = true;
        robot_state_->updateCommandTrajectoryPoint(initial_tp);
    } else {
        RDT_LOG_ERROR(MODULE_NAME, "Initial FK failed.");
        // Continue, but warn: the command pose stays flagged invalid, so Cartesian motion
        // (LIN/CIRC/SPLINE) is refused fail-closed until FK enrichment succeeds.
    }
    planner_->setCurrentState(initial_tp);

    // 4. Start Motion Manager
    if (!motion_manager_->start()) {
        RDT_LOG_CRITICAL(MODULE_NAME, "MotionManager failed to start.");
        return false;
    }

    // 5. Update Global State
    robot_state_->updateRobotMode(RobotMode::Idle);
    robot_state_->updateSystemMessage("System Ready", false);

    // 6. Baseline the realtime-backend selection tracker. main() already built the HAL from the
    // persisted value, so the startup value must not trigger a live switch on the first poll.
    last_applied_realtime_backend_ = robot_state_->getConfigRealtimeBackend();

    RDT_LOG_INFO(MODULE_NAME, "RobotController initialized successfully.");
    return true;
}

void RobotController::update() {
    // 1. Process Feedback from RT Loop
    processMotionFeedback();

    // 2. Process Incoming Network Commands
    processNetworkCommands();

    // 3. Handle Program Execution Logic
    handleProgramExecution();

    // 4. Keep HAL runtime snapshot synchronized
    syncHalRuntimeState();
    
    // 5. Feed the planner to keep the RT buffer full
    planner_->update();
}

void RobotController::processMotionFeedback() {
    TrajectoryPoint feedback;
    // Drain the feedback queue to get the latest state and process all events
    while (motion_manager_->dequeueFeedback(feedback)) {
        // Enrich feedback with Cartesian data for downstream UI/state
        if (solver_) {
            CartPose tcp_actual{};
            if (solveFK(feedback.feedback.joint_actual, tcp_actual)) {
                feedback.feedback.cartesian_actual = tcp_actual;
            }

            CartPose tcp_cmd{};
            if (solveFK(feedback.command.joint_target, tcp_cmd)) {
                feedback.command.cartesian_target = tcp_cmd;
                feedback.command.cartesian_valid = true;
            }
        }

        // Update RobotState source of truth
        robot_state_->updateFeedbackTrajectoryPoint(feedback);

        // Track the executing program line from feedback. Because a motion run is now planned ahead as
        // one continuous chain (dispatchAction), the displayed line must follow the waypoint the robot
        // is physically executing (carried as sequence_index on each point) rather than the planning
        // index. The controller sits in WaitingMotion for the whole physical execution of a chain
        // (Running is only the brief deciding window), so both states must track (REQ-PAUSE-03; the
        // old Running-only gate predated the sequencer's WaitingMotion state and froze the line at
        // the chain's first waypoint). HOLD points (empty RT buffer) carry no meaningful waypoint
        // and must not reset the line. The same value feeds PAUSE/RESUME: the resume re-plan starts
        // from this waypoint (REQ-PAUSE-02).
        if ((exec_state_ == ExecutionState::Running ||
             exec_state_ == ExecutionState::WaitingMotion) &&
            feedback.header.motion_type != MotionType::HOLD) {
            executing_motion_line_ = static_cast<std::int32_t>(feedback.header.sequence_index);
            robot_state_->updateProgramLine(static_cast<int>(feedback.header.sequence_index));
        }

        // Publish the REAL hardware link status (HexaMotion -> Motor Configurator) to the HMI,
        // independent of the active driver. In Simulation the active feedback's hal is Ok (Sim),
        // which would falsely show the Motor Configurator as connected; report the real link instead.
        // Single real-driver read, reused below for both the link-status indicator and the ghost
        // joints (audit F6: this HAL feedback was polled twice per feedback iteration).
        auto real_fb = hw_manager_->getRealDriverFeedback();
        robot_state_->updateRealtimeLinkStatus(
            real_fb.isSuccess() ? real_fb.value().driver_status : HalStatus::NotConnected);

        // GHOST SEMANTICS (Convention B — uniform, mode-independent):
        //   SOLID robot  = the COMMANDED pose (command.joint_target), always.
        //   GHOST        = the PHYSICAL robot (real driver feedback), always.
        // The ghost source is the real driver in BOTH modes: in REAL it is the active driver, in SIM
        // it is the shadow of the physical robot (which stands still while the simulation moves). This
        // replaces the previous per-mode overload of command.joint_target with the real-hardware
        // position; that field now stays the honest command everywhere (see REQ_ghost_semantics_convention_B).
        // On the sim backend the real driver is the VIRTUAL Motor Configurator (REQ-SIMMC-02), so
        // the ghost is live on every backend — same code path, no sim special case.
        if (real_fb.isSuccess()) {
            // Cache the last valid reading so the ghost does not snap to zero on a momentary link drop.
            last_valid_real_joints_ = real_fb.value().joints;
        }
        robot_state_->updateRealHardwareJoints(last_valid_real_joints_);

        // Solid robot: publish the honest command point (command.joint_target already FK-enriched above).
        robot_state_->updateCommandTrajectoryPoint(feedback);

        // Also update the planner's start point for any new trajectories
        planner_->setCurrentState(feedback);

        // Check Diagnostics / Errors
        if (!feedback.diagnostics.isHealthy()) {
            if (feedback.diagnostics.safety != SafetyStatus::Ok) {
                RDT_LOG_ERROR(MODULE_NAME, "Safety Error Detected! Code: {}", static_cast<int>(feedback.diagnostics.safety));
                robot_state_->updateSystemMessage("Safety Error: Motion Stopped", true);
                robot_state_->updateRobotMode(RobotMode::Error);
                stopProgram();
            } else if (feedback.diagnostics.hal == HalStatus::Error_DriveFault ||
                       feedback.diagnostics.hal == HalStatus::Error_CommunicationLost ||
                       feedback.diagnostics.hal == HalStatus::Error_Temperature) {
                 // Only hard HAL errors are fatal. Warning_SyncLost is a recoverable transient
                 // ("lost a packet, interpolation covered it") and must NOT trip Error/stopProgram,
                 // otherwise normal telemetry jitter would block all subsequent motor commands.
                 RDT_LOG_ERROR(MODULE_NAME, "HAL Error Detected! Code: {}", static_cast<int>(feedback.diagnostics.hal));

                 // ===================== TEMPORARY (MKS bring-up) =========================
                 // Hard HAL errors are intentionally NOT treated as fatal for now. During
                 // Motor Configurator integration the link/drive still reports transient
                 // Error_CommunicationLost / Error_DriveFault that would otherwise latch
                 // RobotMode::Error + stopProgram() and block all further commands.
                 // We log + surface the error but do NOT latch Error or stop the program.
                 // TODO(restore): re-enable the two lines below once the MKS HAL link is
                 // stable. This block must be reverted before production.
                 // is_error=false on purpose: latching hasError() here would also block
                 // isRobotReadyForMotion(), which is exactly what we are temporarily lifting.
                 robot_state_->updateSystemMessage("HAL hardware error (non-fatal, TEMPORARY)", false);
                 // robot_state_->updateRobotMode(RobotMode::Error);   // TEMP-DISABLED (MKS bring-up)
                 // stopProgram();                                     // TEMP-DISABLED (MKS bring-up)
                 // =======================================================================
            }
        }
    }
}

void RobotController::processNetworkCommands() {
    auto cmd = robot_state_->getLastReceivedCommand();

    // 1. Emergency Stop (highest priority).
    // Actuation is driven off the LATCHED E-Stop LEVEL, not the transient wire flag. RobotState
    // latches setEStop/resetEStop on the network thread for every received packet, so isEStopActive()
    // is reliable; the wire flag is one-shot on the client and racy across the 4 ms NRT poll (audit
    // F2). Edge detection fires the physical stop/release exactly once per transition — no repeated
    // drive-stop commands or CRITICAL-log spam, and no missed stop if the NRT loop stalls past a
    // client packet. Because actuation follows the latched LEVEL (not the wire flag), any future path
    // that sets the E-Stop level directly would actuate through this same edge (today the only setters
    // are the client command via processNetworkCommand and handleEmergencyStop itself).
    const bool estop_now = robot_state_->isEStopActive();
    if (estop_now && !estop_actuated_) {
        handleEmergencyStop(true);
        estop_actuated_ = true;
    } else if (!estop_now && estop_actuated_) {
        handleEmergencyStop(false);
        estop_actuated_ = false;
    }
    if (estop_now) return;

    // 2. Clear Error — rising edge on the one-shot flag so the hardware recovery (RT reset, drive
    // fault clear) runs once per press instead of on every NRT poll while the flag lingers in the
    // last-received slot (audit F2). A missed edge is benign here: the operator simply presses again.
    if (cmd.clearError && !clear_error_actuated_) {
        handleClearError();
    }
    clear_error_actuated_ = cmd.clearError;

    // 3. Mode Switch (Sim <-> Real)
    // Process each HMI mode request exactly once (monotonic id). A failed REAL attempt stays
    // retryable on the next user toggle instead of being swallowed by a stale request latch.
    if (cmd.modeChangeReqId != last_mode_req_id_) {
        handleModeSwitch(cmd.enableRealMode);
        last_mode_req_id_ = cmd.modeChangeReqId;
    }

    // In Error state, allow only mode switching + clear/reset paths for recovery. The drop must
    // not be silent (mandate: no silent failures; boss report 2026-07-07 - HOME/SET ZERO "did
    // nothing" after a latched error, with zero diagnostics): consume newly arriving service
    // req-ids with an explicit WARN + operator message, so the panel stops resending and the
    // operator learns WHY the command was ignored and how to recover.
    if (robot_state_->hasError()) {
        if (cmd.masteringReqId != 0 && cmd.masteringReqId != last_mastering_req_id_) {
            RDT_LOG_WARN(MODULE_NAME, "Set Zero ignored: ERROR state active. Press CLEAR ERRORS first.");
            robot_state_->updateSystemMessage("Set Zero ignored: clear the ERROR first", true);
            last_mastering_req_id_ = cmd.masteringReqId;
            robot_state_->updateProcessedMasteringReqId(cmd.masteringReqId);
        }
        if (cmd.homingReqId != 0 && cmd.homingReqId != last_homing_req_id_) {
            RDT_LOG_WARN(MODULE_NAME, "Homing ignored: ERROR state active. Press CLEAR ERRORS first.");
            robot_state_->updateSystemMessage("Homing ignored: clear the ERROR first", true);
            last_homing_req_id_ = cmd.homingReqId;
            robot_state_->updateProcessedHomingReqId(cmd.homingReqId);
        }
        return;
    }

    // 4. Program Control — req-id transaction (audit F2, mirrors modeChangeReqId). The client keeps
    // RUN/PAUSE/STOP riding every snapshot with a monotonic programCmdReqId; the controller executes
    // each id exactly once. A lost or superseded packet no longer drops the command (STOP previously
    // had no state latch and could be silently lost across an NRT stall).
    if (cmd.programCmdReqId != 0 && cmd.programCmdReqId != last_program_cmd_req_id_) {
        handleProgramCommand(cmd.programCommand);
        last_program_cmd_req_id_ = cmd.programCmdReqId;
    }

    // 4.1 Jog enable / brakes command
    if (cmd.jogEnableCommand != -1 && cmd.jogEnableCommand != last_jog_enable_cmd_) {
        handleJogEnableCommand(cmd.jogEnableCommand == 1);
        last_jog_enable_cmd_ = cmd.jogEnableCommand;
    }

    // 5. Jogging (incremental)
    NetProtocol::ControlState jogCmd;
    if (robot_state_->getPendingJogCommand(jogCmd)) {
        if (jogCmd.jogHalDirect) {
            handleHalDirectJog(jogCmd);   // HAL overlay → direct point-to-point to the Motor Configurator
        } else {
            handleJogRequest(jogCmd);
        }
        robot_state_->clearPendingJogCommand();
    }
    
    // 6. Update Overrides
    if (cmd.speedOverride >= 0.0) {
        robot_state_->updateSpeedRatio(cmd.speedOverride);
    }
    
    // --- NEW: Calibration & Homing ---
    // Audit F-06: req-id transactions (same pattern as modeChangeReqId). The client resends
    // the command until the processed id is echoed back, the latch executes it exactly once.
    // "Processed" means consumed — executed OR explicitly rejected (the rejection reaches the
    // operator via the system message inside the handler); either way the resend must stop.
    // 7. Mastering
    if (cmd.masteringReqId != 0 && cmd.masteringReqId != last_mastering_req_id_) {
        handleMasteringRequest(cmd);
        last_mastering_req_id_ = cmd.masteringReqId;
        robot_state_->updateProcessedMasteringReqId(cmd.masteringReqId);
    }

    // 8. Homing
    if (cmd.homingReqId != 0 && cmd.homingReqId != last_homing_req_id_) {
        handleHomingRequest(cmd);
        last_homing_req_id_ = cmd.homingReqId;
        robot_state_->updateProcessedHomingReqId(cmd.homingReqId);
    }

    // 8.1 HAL runtime config
    auto hal_cmd = robot_state_->getHalConfigCmd();
    if (hal_cmd.requestId != 0 && hal_cmd.requestId != last_hal_config_req_id_) {
        (void)hw_manager_->setHalConfig(hal_cmd);
        last_hal_config_req_id_ = hal_cmd.requestId;
    }

    // 8.2 Realtime-backend selection (HAL panel dropdown, riding the config transaction).
    // Applied exactly once per changed value; the tracker advances even on refusal so a rejected
    // switch does not retry every NRT cycle — the operator re-selects after fixing the cause.
    const std::string selected_backend = robot_state_->getConfigRealtimeBackend();
    if (selected_backend != last_applied_realtime_backend_) {
        handleRealtimeBackendSelection(selected_backend);
        last_applied_realtime_backend_ = selected_backend;
    }

    // --- NEW: Trajectory Preview ---
    // 9. Program Update (triggers preview)
    if (cmd.programUpdateReqId != last_processed_prog_update_id_) {
        handleProgramUpdateRequest(cmd);
        last_processed_prog_update_id_ = cmd.programUpdateReqId;
    }
}

void RobotController::handleEmergencyStop(bool active) {
    if (active) {
        RDT_LOG_CRITICAL(MODULE_NAME, "E-STOP REQUESTED!");
        motion_manager_->emergencyStop();
        // F-13: freezing our RT stream is not enough for backends that profile motion in
        // firmware (MKS) — actively stop the drives too. Goes through the configurator's
        // ownership gate (no bypass by project decision); with owner=UI the configurator
        // rejects it and its own GUI E-Stop is the physical stop.
        if (auto hw_res = hw_manager_->emergencyStopAll(); hw_res.isError()) {
            RDT_LOG_WARN(MODULE_NAME, "E-Stop hardware forward failed: {}", ToString(hw_res.error()));
        }
        robot_state_->updateEStopState(true);
        robot_state_->updateJogEnabled(false);
        (void)hw_manager_->setBrakeState(true);
        robot_state_->updateRobotMode(RobotMode::EStop);
        stopProgram();
    } else {
        RDT_LOG_INFO(MODULE_NAME, "E-Stop Reset Request.");
        robot_state_->updateEStopState(false);
        // Does not automatically clear Error, separate ClearError required
    }
}

void RobotController::handleClearError() {
    RDT_LOG_INFO(MODULE_NAME, "Clear Error Request.");
    motion_manager_->reset(); // Reset RT buffers

    auto current_tp = robot_state_->getFeedbackTrajectoryPoint();
    planner_->setCurrentState(current_tp);

    // Propagate the clear all the way down to the hardware so latched drive faults
    // (e.g. MKS stall/protection) are actually reset, not just the RT buffers.
    (void)hw_manager_->clearDriveErrors();

    robot_state_->updateSystemMessage("Ready", false);
    robot_state_->updateJogEnabled(false);
    (void)hw_manager_->setBrakeState(true);
    robot_state_->updateRobotMode(RobotMode::Idle);
}

void RobotController::handleJogEnableCommand(bool enabled) {
    // Any arm/disarm toggle invalidates the maintained Cartesian jog target so the next jog
    // re-syncs from the live robot pose (no jump from a stale accumulated target).
    jog_active_ = false;

    if (enabled) {
        if (robot_state_->isEStopActive() || robot_state_->hasError()) {
            robot_state_->updateJogEnabled(false);
            robot_state_->updateSystemMessage("Cannot enable JOG: clear E-STOP/ERROR first", false);
            return;
        }

        if (!robot_state_->isSimulated()) {
            auto diag = robot_state_->getFeedbackTrajectoryPoint().diagnostics;
            if (diag.hal != HalStatus::Ok) {
                robot_state_->updateJogEnabled(false);
                robot_state_->updateSystemMessage("Cannot enable JOG: hardware not ready", false);
                return;
            }
        }

        auto brake_res = hw_manager_->setBrakeState(false);
        if (brake_res.isError()) {
            robot_state_->updateJogEnabled(false);
            robot_state_->updateSystemMessage("Cannot enable JOG: brake release failed", false);
            return;
        }

        robot_state_->updateJogEnabled(true);
        robot_state_->updateSystemMessage("JOG enabled", false);
        return;
    }

    (void)hw_manager_->setBrakeState(true);
    robot_state_->updateJogEnabled(false);
    robot_state_->updateSystemMessage("JOG disabled", false);
}

void RobotController::handleModeSwitch(bool enable_real_mode) {
    HalMode target_mode = enable_real_mode ? HalMode::Realtime : HalMode::Simulation;
    
    if (enable_real_mode && hw_manager_->getMode() == HalMode::Simulation) {
        // Try to sync sim to real before switching
        auto res = hw_manager_->syncSimulationToReal();
        if (res.isError()) {
            RDT_LOG_WARN(MODULE_NAME, "Sync failed: {}. Attempting to re-init Real driver...", ToString(res.error()));
            
            // Try to re-initialize the real driver if it failed previously
            auto reinit_res = hw_manager_->reinitializeRealtimeDriver();
            if (reinit_res.isError()) {
                 RDT_LOG_ERROR(MODULE_NAME, "Real driver re-init failed: {}", ToString(reinit_res.error()));
                 robot_state_->updateSystemMessage("Real Driver Init Failed", true);
                 return;
            }
            
            // Retry sync after re-init
            res = hw_manager_->syncSimulationToReal();
            if (res.isError()) {
                 RDT_LOG_ERROR(MODULE_NAME, "Sync failed after re-init: {}", ToString(res.error()));
                 robot_state_->updateSystemMessage("Sync Failed: Check Connection", true);
                 return;
            }
        }
    }

    auto res = hw_manager_->setMode(target_mode);
    if (res.isSuccess()) {
        robot_state_->updateSimulatedState(target_mode == HalMode::Simulation);
        RDT_LOG_INFO(MODULE_NAME, "Mode switched to {}", enable_real_mode ? "REAL" : "SIM");

        // FORCE SYNC: Immediately read the new state to prevent planner jumping to zero
        auto read_res = hw_manager_->read();
        if (read_res.isSuccess()) {
            // Reset motion manager buffers to clear any old mode data
            motion_manager_->reset();
            
            // Update planner start state to current actual position
            TrajectoryPoint tp;
            tp.command.joint_target = read_res.value().joints;
            // Solve FK for completeness; on failure the pose stays flagged invalid (fail-closed
            // for the next Cartesian plan) instead of riding along as a silent zero.
            if (solver_) {
                tp.command.cartesian_valid = solveFK(tp.command.joint_target, tp.command.cartesian_target);
            }
            // Force set the planner state
            planner_->setCurrentState(tp);
            
            // Also update robot state so UI sees the jump immediately if any
            robot_state_->updateFeedbackTrajectoryPoint(tp);
            robot_state_->updateCommandTrajectoryPoint(tp);
            
            RDT_LOG_INFO(MODULE_NAME, "State synchronized after mode switch.");
        } else {
            RDT_LOG_ERROR(MODULE_NAME, "Failed to sync state after mode switch!");
        }

    } else {
        RDT_LOG_ERROR(MODULE_NAME, "Mode switch failed: {}", ToString(res.error()));
        robot_state_->updateSystemMessage("Mode Switch Failed", true);
    }
}

void RobotController::handleJogRequest(const NetProtocol::ControlState& cmd) {
    if (!isRobotReadyForMotion()) return;

    if (!robot_state_->isJogEnabled()) {
        RDT_LOG_WARN(MODULE_NAME, "Jog rejected: JOG is not enabled.");
        return;
    }
    
    if (exec_state_ != ExecutionState::Stopped) {
        RDT_LOG_WARN(MODULE_NAME, "Cannot jog while program is running.");
        return;
    }

    // Apply Jog Logic
    if (cmd.jogAxis < 0 || cmd.jogAxis >= static_cast<int>(ROBOT_AXES_COUNT)) {
        return;
    }

    RDT_LOG_DEBUG(MODULE_NAME,
                  "Jog cmd: frame={}, axis={}, inc={}, toolId={}, baseId={}",
                  static_cast<int>(cmd.jogFrame), cmd.jogAxis, cmd.jogIncrement,
                  cmd.activeToolId, cmd.activeBaseId);

    auto current_fb = robot_state_->getFeedbackTrajectoryPoint();

    if (cmd.jogFrame == NetProtocol::JogFrame::JOINT) {
        TrajectoryPoint tp;
        tp.header.motion_type = MotionType::JOINT;
        tp.command.speed_ratio = robot_state_->getSpeedRatio(); // jog speed from UI override (passed through to segment_target)
        tp.command.acceleration_ratio = 0.5;
        tp.command.joint_target = current_fb.feedback.joint_actual;

        auto axis_id = static_cast<AxisId>(cmd.jogAxis);
        tp.command.joint_target[axis_id].position += RDT::Degrees(cmd.jogIncrement);

        // retargetJog (not addTargetWaypoint): each jog supersedes the previous so manual jogging
        // never accumulates behind the MotionManager's per-segment completion gate.
        auto plan_res = planner_->retargetJog(tp);
        if (plan_res.isError()) {
            RDT_LOG_WARN(MODULE_NAME, "Jog planning failed. Error: {}", static_cast<int>(plan_res.error()));
        } else {
            RDT_LOG_INFO(MODULE_NAME, "Jog planned successfully to axis {}: {}",
                         cmd.jogAxis, tp.command.joint_target[axis_id].position.value());
        }
        return;
    }

    // Cartesian jog (WORLD / BASE / TOOL), KUKA-style. The increment accumulates on a MAINTAINED
    // TCP target (jog_tcp_world_) and IK seed (jog_seed_joints_), rotating ABOUT THE TCP so the TCP
    // stays fixed during an orientation jog. A single IK with a continuity guard rejects
    // configuration flips (no random large moves), and the step executes as a JOINT move to the IK
    // solution (no per-interpolation Cartesian IK, which is what jumped near singularities).
    // Re-sync the maintained target if the seed drifts this far from the live joints (external move,
    // e.g. a program or homing ran). This is NOT a jog block -- it only re-anchors the accumulation
    // point; the jog itself is never rejected.
    static constexpr double kJogResyncDeg = 120.0;

    const auto frames = robot_state_->getFrameSet();
    const int tool_id = (cmd.activeToolId >= 0) ? cmd.activeToolId : robot_state_->getActiveToolId();
    const int base_id = (cmd.activeBaseId >= 0) ? cmd.activeBaseId : robot_state_->getActiveBaseId();

    CartPose tool_offset{};
    CartPose base_offset{};
    for (const auto& tool : frames.tools) {
        if (tool.id == tool_id) { tool_offset = tool.offset; break; }
    }
    for (const auto& base : frames.bases) {
        if (base.id == base_id) { base_offset = base.offset; break; }
    }

    const AxisSet current_joints = current_fb.feedback.joint_actual;

    // Largest absolute per-joint difference (degrees) between two joint sets.
    auto maxJointDeltaDeg = [](const AxisSet& a, const AxisSet& b) {
        double m = 0.0;
        for (std::size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            const auto id = static_cast<AxisId>(i);
            m = std::max(m, std::fabs(a[id].position.value() - b[id].position.value()));
        }
        return m;
    };

    // (Re)sync the maintained jog target when starting fresh (re-arm) or after an external move
    // drifted the seed away from the live robot. Sync from the live joints (robot is at rest at a
    // fresh jog start), expressing the TCP via the active tool so rotations pivot about the TCP.
    const bool resync = !jog_active_ || (maxJointDeltaDeg(current_joints, jog_seed_joints_) > kJogResyncDeg);
    if (resync) {
        CartPose flange_now{};
        if (!solveFK(current_joints, flange_now)) {
            RDT_LOG_WARN(MODULE_NAME, "FK failed for current joints. Cartesian jog skipped.");
            return;
        }
        jog_seed_joints_ = current_joints;
        jog_tcp_world_ = FrameTransformer::calculateTcpInWorld(flange_now, tool_offset);
        jog_active_ = true;
    }

    // Apply the increment to the maintained TCP target. A pure-rotation delta keeps the TCP position
    // fixed (rotation about the TCP); translation moves the TCP along the chosen frame axes.
    CartPose tcp_target = jog_tcp_world_;
    if (cmd.jogFrame == NetProtocol::JogFrame::TOOL) {
        // Tool frame is intrinsic: post-multiply the delta onto the TCP (= tool) frame.
        CartPose delta_tool{};
        switch (cmd.jogAxis) {
            case 0: delta_tool.x += RDT::Millimeters(cmd.jogIncrement); break;
            case 1: delta_tool.y += RDT::Millimeters(cmd.jogIncrement); break;
            case 2: delta_tool.z += RDT::Millimeters(cmd.jogIncrement); break;
            case 3: delta_tool.rx += RDT::Degrees(cmd.jogIncrement); break;
            case 4: delta_tool.ry += RDT::Degrees(cmd.jogIncrement); break;
            case 5: delta_tool.rz += RDT::Degrees(cmd.jogIncrement); break;
            default: break;
        }
        tcp_target = FrameTransformer::composePoses(tcp_target, delta_tool);
    } else {
        // World/base frame: translation along base axes; orientation as an extrinsic rotation about
        // the base axis (pre-multiply), which keeps the TCP position fixed.
        CartPose tcp_base = FrameTransformer::applyBaseInverseTransform(base_offset, tcp_target);
        switch (cmd.jogAxis) {
            case 0: tcp_base.x += RDT::Millimeters(cmd.jogIncrement); break;
            case 1: tcp_base.y += RDT::Millimeters(cmd.jogIncrement); break;
            case 2: tcp_base.z += RDT::Millimeters(cmd.jogIncrement); break;
            case 3:
            case 4:
            case 5:
                tcp_base = FrameTransformer::rotateAboutWorldAxis(
                    tcp_base, cmd.jogAxis - 3, RDT::Degrees(cmd.jogIncrement));
                break;
            default: break;
        }
        tcp_target = FrameTransformer::applyBaseTransform(base_offset, tcp_base);
    }

    // TCP target -> flange target for IK (the kinematic chain solves to the flange).
    const CartPose flange_target = FrameTransformer::calculateFlangeInWorld(tcp_target, tool_offset);

    // One IK from the maintained seed for configuration continuity.
    auto ik_res = solveIK(flange_target, jog_seed_joints_);
    if (ik_res.isError()) {
        RDT_LOG_WARN(MODULE_NAME,
                     "Jog IK failed (unreachable/singular). Step skipped; jog target not advanced.");
        // Surface the reason so the operator sees WHY the jog stopped instead of it silently
        // sticking (typically a joint reached its software limit and the orientation/position
        // beyond it is unreachable).
        robot_state_->updateSystemMessage("Jog stopped: axis limit / target unreachable", false);
        return; // do not advance the maintained target on failure
    }
    const AxisSet ik_joints = ik_res.value();

    // No jog blocking: the seeded IK keeps the configuration continuous; we only log the step size
    // for diagnostics. (Singularity robustness is the job of the velocity-level solver; a hard
    // step-size rejection is the wrong tool and was removed.)
    const double ik_step_deg = maxJointDeltaDeg(ik_joints, jog_seed_joints_);

    // Commit: advance the maintained target + seed, then execute as a smooth JOINT move to the IK
    // solution. JOINT (not LIN) avoids per-interpolation Cartesian IK, the source of the random
    // moves near singularities.
    jog_tcp_world_ = tcp_target;
    jog_seed_joints_ = ik_joints;

    TrajectoryPoint tp;
    tp.header.motion_type = MotionType::JOINT;
    tp.command.speed_ratio = robot_state_->getSpeedRatio();
    tp.command.acceleration_ratio = 0.5;
    tp.command.joint_target = ik_joints;
    tp.command.cartesian_target = tcp_target; // informational; UI re-derives display from FK
    tp.command.cartesian_valid = true;        // real pose: the maintained Cartesian jog target

    RDT_LOG_DEBUG(MODULE_NAME,
                  "Jog commit: TCP world(mm,deg)=({}, {}, {}, {}, {}, {}), IK step={:.2f} deg",
                  tcp_target.x.value(), tcp_target.y.value(), tcp_target.z.value(),
                  tcp_target.rx.value(), tcp_target.ry.value(), tcp_target.rz.value(), ik_step_deg);

    // retargetJog (supersede, not append): keeps jog responsive instead of draining a
    // completion-gated queue.
    auto plan_res = planner_->retargetJog(tp);
    if (plan_res.isError()) {
        RDT_LOG_WARN(MODULE_NAME, "Cartesian jog planning failed. Error: {}", static_cast<int>(plan_res.error()));
    }
}

void RobotController::handleHalDirectJog(const NetProtocol::ControlState& cmd) {
    // HAL overlay jog: drive the Motor Configurator (real driver) directly, point-to-point,
    // independent of the SIM/REAL view and the planner. E-Stop/error are already gated upstream;
    // the overlay arms jog via setJogEnabled, so we only require jog enabled + a valid axis.
    if (!robot_state_->isJogEnabled()) {
        RDT_LOG_WARN(MODULE_NAME, "HAL jog rejected: JOG is not enabled.");
        return;
    }
    if (cmd.jogAxis < 0 || cmd.jogAxis >= static_cast<int>(ROBOT_AXES_COUNT)) {
        return;
    }

    // Sim backend, REAL view ONLY (REQ-SIMMC-06): there the virtual MC is the ACTIVE
    // stream-followed driver, so a point-to-point jump would be overwritten by the 4 ms hold
    // stream within one cycle — ride the production JOINT jog path instead (limits + profile
    // respected). In SIM view the virtual MC receives no stream (keep-alive skipped,
    // REQ-SIMMC-04), so the direct MC jog below works uniformly across all backends.
    if (hw_manager_->getRealtimeInterfaceType() == InterfaceConfig::RealtimeInterfaceType::None &&
        hw_manager_->getMode() == HalMode::Realtime) {
        NetProtocol::ControlState joint_jog = cmd;
        joint_jog.jogFrame = NetProtocol::JogFrame::JOINT;   // HAL jog is per-axis by definition
        handleJogRequest(joint_jog);
        return;
    }

    // Carry the UI SPEED% so the MKS driver scales this HAL-direct jog's axis speed.
    auto res = hw_manager_->jogRealIncremental(cmd.jogAxis, cmd.jogIncrement, robot_state_->getSpeedRatio());
    if (res.isError()) {
        RDT_LOG_WARN(MODULE_NAME, "HAL jog failed for axis {}: {}", cmd.jogAxis, ToString(res.error()));
        robot_state_->updateSystemMessage("HAL jog failed (Motor Configurator not connected?)", false);
    }
}

void RobotController::handleRealtimeBackendSelection(const std::string& backend) {
    // Map the wire string to the HAL type. "sim" (and anything unknown) resolves to the internal
    // simulator — the safe default, mirroring the startup selection in main().
    InterfaceConfig::RealtimeInterfaceType type = InterfaceConfig::RealtimeInterfaceType::None;
    if (backend == "mks_tcp") {
        type = InterfaceConfig::RealtimeInterfaceType::MksTcp;
    } else if (backend == "udp") {
        // UDP live switching is deliberately deferred (boss decision 2026-07-06: "решим потом").
        // Selecting it takes effect on the controller's next start via the persisted config.
        RDT_LOG_WARN(MODULE_NAME,
                     "Realtime backend 'udp' selected: live switching is not supported yet; "
                     "the selection is persisted and applies on the next controller start.");
        robot_state_->updateSystemMessage(
            "UDP backend: applies on next controller start (live switch not supported yet)", false);
        return;
    } else if (backend != "sim") {
        RDT_LOG_WARN(MODULE_NAME,
                     "Unknown realtime backend '{}' selected. Resolving to Simulation (fail-safe).",
                     backend);
    }

    auto res = hw_manager_->setRealtimeInterfaceType(type);
    if (res.isError()) {
        RDT_LOG_WARN(MODULE_NAME, "Realtime backend switch to '{}' refused: {}.",
                     backend, ToString(res.error()));
        robot_state_->updateSystemMessage(
            "Backend switch refused: leave REAL mode first, then re-select", true);
        return;
    }
    RDT_LOG_INFO(MODULE_NAME, "Realtime backend switched to '{}'.", backend);
    robot_state_->updateSystemMessage(
        backend == "sim" ? "HAL backend: SIMULATION (internal stub)"
                         : "HAL backend: MKS TCP (use CONNECT to open the transport)",
        false);
}

void RobotController::handleProgramCommand(int command) {
    // 0=NOP, 1=RUN, 2=PAUSE, 3=STOP
    switch (command) {
        case 1: startProgram(); break;
        case 2: pauseProgram(); break;
        case 3: stopProgram(); break;
        default: break;
    }
}

void RobotController::handleMasteringRequest(const NetProtocol::ControlState& cmd) {
    // Sim backend: the service target is the virtual MC (real-first routing, REQ-SIMMC-07).
    // The operation completes instantly; a hold-target resync (MotionManager::reset(), the same
    // feedback re-adoption used on a mode switch) is needed ONLY in REAL view, where the virtual
    // MC is also the ACTIVE driver and the RT loop would stream the PREVIOUS hold target right
    // back every 4 ms (set-zero would read as a no-op). In SIM view the virtual MC receives no
    // stream (keep-alive skipped, REQ-SIMMC-04), so the result sticks without a resync.
    const bool virtual_mc_is_active =
        (hw_manager_->getRealtimeInterfaceType() == InterfaceConfig::RealtimeInterfaceType::None) &&
        (hw_manager_->getMode() == HalMode::Realtime);

    // "Set Zero All" — single atomic command (the HAL panel sends one masterAxisId=sentinel, not a
    // loop of per-axis requests that would collapse in the one-shot field). Drive all axes at once.
    if (cmd.masterAxisId == NetProtocol::kMasterAllAxesId) {
        if (!isRobotReadyForMotion()) {
            RDT_LOG_WARN(MODULE_NAME, "Set Zero All rejected: robot not ready (E-Stop/Error).");
            robot_state_->updateSystemMessage("Set Zero All rejected: E-Stop/Error active", true);
            return;
        }
        RDT_LOG_INFO(MODULE_NAME, "Mastering request: ALL axes (SetZeroAll).");
        auto res = hw_manager_->zeroAllAxes();
        if (res.isError()) {
            RDT_LOG_ERROR(MODULE_NAME, "Failed to send SetZeroAll: {}", ToString(res.error()));
            robot_state_->updateSystemMessage("Set Zero All Failed", true);
        } else if (virtual_mc_is_active) {
            motion_manager_->reset();   // adopt the re-zeroed feedback as the new RT hold target
            RDT_LOG_INFO(MODULE_NAME, "SetZeroAll applied to the virtual MC; RT hold resynced.");
            robot_state_->updateSystemMessage("Set Zero All done (virtual MC)", false);
        } else {
            // Honest wording (audit F-01): success here only means the request left the
            // socket; backend acceptance is reported separately via last_ipc_error.
            RDT_LOG_INFO(MODULE_NAME, "SetZeroAll request sent to Motor Configurator.");
            robot_state_->updateSystemMessage("Set Zero All sent to Motor Configurator", false);
        }
        return;
    }
    if (cmd.masterAxisId < 0 || cmd.masterAxisId >= ROBOT_AXES_COUNT) return;
    if (!isRobotReadyForMotion()) {
        RDT_LOG_WARN(MODULE_NAME, "Set Zero rejected: robot not ready (E-Stop/Error).");
        robot_state_->updateSystemMessage("Set Zero rejected: E-Stop/Error active", true);
        return;
    }

    auto axis_id = static_cast<AxisId>(cmd.masterAxisId);
    RDT_LOG_INFO(MODULE_NAME, "Mastering request for axis {}", cmd.masterAxisId);

    auto res = hw_manager_->zeroAxis(axis_id);
    if (res.isError()) {
        RDT_LOG_ERROR(MODULE_NAME, "Failed to send Set Zero for axis {}: {}", cmd.masterAxisId, ToString(res.error()));
        robot_state_->updateSystemMessage("Mastering Failed", true);
    } else if (virtual_mc_is_active) {
        motion_manager_->reset();   // adopt the re-zeroed feedback as the new RT hold target
        RDT_LOG_INFO(MODULE_NAME, "Set Zero for axis {} applied to the virtual MC; RT hold resynced.", cmd.masterAxisId);
        robot_state_->updateSystemMessage("Set Zero done (virtual MC)", false);
    } else {
        RDT_LOG_INFO(MODULE_NAME, "Set Zero for axis {} sent to Motor Configurator.", cmd.masterAxisId);
        robot_state_->updateSystemMessage("Set Zero sent to Motor Configurator", false);
    }
}

void RobotController::handleHomingRequest(const NetProtocol::ControlState& cmd) {
    if (robot_state_->isEStopActive() || robot_state_->hasError()) {
        RDT_LOG_WARN(MODULE_NAME, "Cannot start homing: clear E-STOP/ERROR first.");
        return;
    }

    // Homing is forwarded to the Motor Configurator (real driver) and is meaningful whenever it is
    // connected, regardless of the SIM/REAL view mode — the HAL panel drives the MC directly. The
    // internal sim never homed, so the old "switch to REAL first" gate is dropped.

    stopProgram();
    const int axis_id = cmd.homingAxisId;
    auto res = hw_manager_->requestHoming(axis_id);
    if (res.isError()) {
        robot_state_->updateSystemMessage("HAL homing request failed", false);
        RDT_LOG_ERROR(MODULE_NAME, "HAL homing request failed for axis {}: {}", axis_id, ToString(res.error()));
        return;
    }

    if (hw_manager_->getRealtimeInterfaceType() == InterfaceConfig::RealtimeInterfaceType::None) {
        // Sim backend: the virtual MC homed instantly (axes at logical zero, REQ-SIMMC-07). The
        // RT-hold resync is needed only in REAL view, where the virtual MC is also the ACTIVE
        // driver — otherwise the 4 ms hold stream commands the axes right back to the pre-homing
        // pose. In SIM view the virtual MC receives no stream (keep-alive skipped, REQ-SIMMC-04).
        if (hw_manager_->getMode() == HalMode::Realtime) {
            motion_manager_->reset();
            RDT_LOG_INFO(MODULE_NAME, "Virtual MC homing complete for axis {}; RT hold resynced.", axis_id);
        } else {
            RDT_LOG_INFO(MODULE_NAME, "Virtual MC homing complete for axis {}.", axis_id);
        }
        robot_state_->updateSystemMessage("Homing complete (virtual MC)", false);
        return;
    }

    // Honest wording (audit F-01): only the socket send is confirmed here; the configurator
    // owns the homing sequence and reports progress via supervisor_status / last_ipc_error.
    robot_state_->updateSystemMessage("Homing request sent to Motor Configurator", false);
}

void RobotController::handleProgramUpdateRequest(const NetProtocol::ControlState& cmd) {
    // This is called when RobotState->processNetworkCommand has already updated the program
    // We just need to trigger the preview generation
    if (!planner_) return;

    RDT_LOG_INFO(MODULE_NAME, "New program version received. Generating trajectory preview...");
    // Align the preview start with the physical robot: joints from feedback, pose from fresh FK.
    // On FK failure the pose is flagged invalid and the planner's own documented fallback adopts
    // the feedback pose (single adoption point — no duplicated fallback here).
    auto current_fb = robot_state_->getFeedbackTrajectoryPoint();
    current_fb.command.joint_target = current_fb.feedback.joint_actual;
    CartPose tcp_actual{};
    if (solveFK(current_fb.feedback.joint_actual, tcp_actual)) {
        current_fb.command.cartesian_target = tcp_actual;
        current_fb.command.cartesian_valid = true;
    } else {
        current_fb.command.cartesian_valid = false;
    }
    planner_->setCurrentState(current_fb);
    auto program = robot_state_->getLoadedProgram();

    // Arm the program sequencer with the new program. Fail-closed: an invalid program (empty,
    // duplicate label id, unresolved jump target, or a step type this build cannot execute) is NOT
    // armed and is surfaced to the operator, so a later RUN is refused. The motion preview below is
    // still generated — it only visualises motion steps and is independent of control-flow validity.
    if (auto load_res = sequencer_.load(program); load_res.isError()) {
        RDT_LOG_ERROR(MODULE_NAME, "Program rejected by sequencer: {}", ToString(load_res.error()));
        robot_state_->updateSystemMessage("Program rejected: " + ToString(load_res.error()), true);
    } else {
        // A newly loaded program has no execution history: load() reset the sequencer, so publish the
        // cleared annotation now (parallels startProgram's reset publish). Without this the pendant's
        // execution strip would keep showing the PREVIOUS program's registers/branch until the next
        // RUN, i.e. stale state against the freshly loaded lines.
        robot_state_->updateProgramExecution(sequencer_.registers(), sequencer_.lastBranchLine(),
                                             sequencer_.lastBranchTaken());
    }

    auto preview_path = planner_->generatePreviewPath(program);
    robot_state_->updateTrajectory(preview_path);
}

void RobotController::syncHalRuntimeState() {
    // Types are now unified — direct assignment, no mapping needed.
    const auto current = hw_manager_->getCurrentHalState();
    robot_state_->updateHalConfigCurrent(current);
}

void RobotController::startProgram() {
    if (!isRobotReadyForMotion()) return;

    if (exec_state_ == ExecutionState::Paused) {
        // RESUME: restore the phase we paused from. A frozen wait resumes from its remaining time
        // (wait_duration_/wait_timeout_ already hold the remainder captured at pause).
        exec_state_ = state_before_pause_;
        if (exec_state_ == ExecutionState::WaitingTime || exec_state_ == ExecutionState::WaitingDI) {
            wait_start_time_ = std::chrono::steady_clock::now();
        } else if (exec_state_ == ExecutionState::WaitingMotion) {
            // The pause hold aborted the planned chain (overrideTrajectory cleared it), so re-plan
            // the REMAINING part of the interrupted chain from the position the robot paused at:
            // the waypoint being executed at pause time (executing_motion_line_, tracked from
            // feedback) and every waypoint after it (REQ-PAUSE-02). The sequencer consumed the
            // whole chain at dispatch, so it is NOT advanced here — it stays AwaitingMotion and
            // completes only when this re-planned remainder physically finishes. The previous
            // behaviour (onActionCompleted + continue with the next step) silently dropped every
            // remaining waypoint, so RESUME drove the robot to an unrelated program target.
            if (active_motion_chain_.empty()) {
                // No chain context (must not happen: WaitingMotion is only entered by a successful
                // dispatch). Fail visible and stop instead of guessing a motion target.
                RDT_LOG_ERROR(MODULE_NAME,
                    "Resume requested from WaitingMotion but no interrupted chain is stored. "
                    "Stopping the program; re-run it from the start.");
                robot_state_->updateSystemMessage("Resume failed: no interrupted motion context", true);
                stopProgram();
                return;
            }
            const std::int32_t chain_size = static_cast<std::int32_t>(active_motion_chain_.size());
            const std::int32_t offset = std::clamp(
                executing_motion_line_ - active_chain_first_line_, std::int32_t{0}, chain_size - 1);
            const std::vector<NetProtocol::ProgramStepStruct> remaining(
                active_motion_chain_.begin() + offset, active_motion_chain_.end());
            const std::int32_t resume_line = active_chain_first_line_ + offset;
            RDT_LOG_INFO(MODULE_NAME,
                "Resuming interrupted motion chain: {} of {} waypoint(s) remain, from line {}.",
                remaining.size(), active_motion_chain_.size(), resume_line);
            robot_state_->updateProgramLine(resume_line);
            if (!planChainSteps(remaining, resume_line)) {
                return; // planChainSteps faulted the program with an operator-visible reason
            }
            exec_state_ = ExecutionState::WaitingMotion;
        }
        robot_state_->updateRobotMode(RobotMode::Running);
        RDT_LOG_INFO(MODULE_NAME, "Resuming program.");
        return;
    }

    // Fresh RUN: require a valid, armed program (the sequencer fail-closes invalid programs at load).
    if (!sequencer_.isLoaded()) {
        RDT_LOG_WARN(MODULE_NAME, "Cannot start: no valid program is loaded.");
        robot_state_->updateSystemMessage("Cannot start: no valid program loaded", true);
        return;
    }
    sequencer_.reset();
    // Fresh RUN: no interrupted-chain context may survive from a previous run (REQ-PAUSE-05).
    active_motion_chain_.clear();
    active_chain_first_line_ = -1;
    executing_motion_line_ = -1;
    // Fresh RUN: publish the cleared annotation immediately so the pendant never shows the previous
    // run's registers/branch against the new run's first lines.
    robot_state_->updateProgramExecution(sequencer_.registers(), sequencer_.lastBranchLine(),
                                         sequencer_.lastBranchTaken());
    waiting_di_line_ = -1;
    exec_state_ = ExecutionState::Running;
    robot_state_->updateRobotMode(RobotMode::Running);
    RDT_LOG_INFO(MODULE_NAME, "Starting program: {}", robot_state_->getLoadedProgram().name);
}

void RobotController::stopProgram() {
    if (exec_state_ != ExecutionState::Stopped) {
        exec_state_ = ExecutionState::Stopped;
        // STOP invalidates any interrupted-chain context (REQ-PAUSE-05): after a STOP the only way
        // forward is a fresh RUN, never a resume of stale motion.
        active_motion_chain_.clear();
        active_chain_first_line_ = -1;
        executing_motion_line_ = -1;

        const TrajectoryPoint hold_tp = physicalHoldPoint();
        // Do not swallow the result on this safety path. The physical halt is already done by
        // overrideTrajectory -> MotionManager::reset() (queues cleared, robot held); a failure here
        // only means the follow-up hold segment could not be replanned, which is non-fatal but must
        // be visible, not hidden behind a (void) cast.
        if (auto res = planner_->overrideTrajectory(hold_tp); res.isError()) {
            RDT_LOG_ERROR(MODULE_NAME,
                "stopProgram: hold replan failed (planner error {}). Motion is already stopped and held "
                "by MotionManager::reset(); the robot is safe, but no new hold segment was planned.",
                static_cast<int>(res.error()));
        }

        robot_state_->updateRobotMode(RobotMode::Idle);
        RDT_LOG_INFO(MODULE_NAME, "Program stopped.");
    }
}

void RobotController::pauseProgram() {
    // Pause is meaningful only while actively running or servicing a blocking action.
    if (exec_state_ != ExecutionState::Running &&
        exec_state_ != ExecutionState::WaitingMotion &&
        exec_state_ != ExecutionState::WaitingTime &&
        exec_state_ != ExecutionState::WaitingDI) {
        return;
    }

    // Remember the phase so RESUME restores it, and freeze an active wait to its remaining time so it
    // does not keep counting down while paused.
    state_before_pause_ = exec_state_;
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - wait_start_time_).count();
    if (exec_state_ == ExecutionState::WaitingTime) {
        const double remaining = wait_duration_.value() - elapsed;
        wait_duration_ = Seconds{remaining > 0.0 ? remaining : 0.0};
    } else if (exec_state_ == ExecutionState::WaitingDI && wait_timeout_.value() > 0.0) {
        const double remaining = wait_timeout_.value() - elapsed;
        wait_timeout_ = Seconds{remaining > 0.0 ? remaining : 0.0};
    }

    exec_state_ = ExecutionState::Paused;

    const TrajectoryPoint hold_tp = physicalHoldPoint();
    // Same safety-path rule as stopProgram(): surface a replan failure instead of discarding it.
    if (auto res = planner_->overrideTrajectory(hold_tp); res.isError()) {
        RDT_LOG_ERROR(MODULE_NAME,
            "pauseProgram: hold replan failed (planner error {}). Motion is already stopped and held "
            "by MotionManager::reset(); the robot is safe, but no new hold segment was planned.",
            static_cast<int>(res.error()));
    }

    robot_state_->updateRobotMode(RobotMode::Paused);
    RDT_LOG_INFO(MODULE_NAME, "Program paused.");
}

void RobotController::handleProgramExecution() {
    switch (exec_state_) {
        case ExecutionState::Running:
            // Idle planner and nothing pending: ask the sequencer for the next action.
            driveSequencer();
            break;

        case ExecutionState::WaitingMotion:
            // Wait for the planned motion chain to finish, then decide the next step.
            if (planner_->isTaskFinished()) {
                // The chain is physically complete: drop the resume context (nothing left to
                // resume) before advancing the sequencer.
                active_motion_chain_.clear();
                active_chain_first_line_ = -1;
                sequencer_.onActionCompleted();
                exec_state_ = ExecutionState::Running;
                driveSequencer();
            }
            break;

        case ExecutionState::WaitingTime: {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed =
                std::chrono::duration_cast<std::chrono::duration<double>>(now - wait_start_time_).count();
            if (elapsed >= wait_duration_.value()) {
                sequencer_.onActionCompleted();
                exec_state_ = ExecutionState::Running;
                driveSequencer();
            }
            break;
        }

        case ExecutionState::WaitingDI: {
            // The sequencer owns no clock, so enforce the WaitDI timeout here (0 = wait indefinitely).
            const auto now = std::chrono::steady_clock::now();
            const double elapsed =
                std::chrono::duration_cast<std::chrono::duration<double>>(now - wait_start_time_).count();
            if (wait_timeout_.value() > 0.0 && elapsed >= wait_timeout_.value()) {
                faultProgram("WaitDI timeout: the expected digital input was not received in time");
                break;
            }
            // Poll the input through the sequencer: if satisfied it self-completes and returns the next
            // action (dispatched here, leaving WaitingDI); otherwise it re-arms WaitForInput and we stay.
            driveSequencer();
            break;
        }

        default:
            // Stopped / Paused: nothing to execute.
            break;
    }
}

void RobotController::driveSequencer() {
    // Distil the DI snapshot from the circulating feedback: IO rides the same struct as motion state,
    // so the branch/wait decision uses one coherent sample instant (see REQ_program_sequencer.md §3.3).
    WorldSample world;
    world.digital_inputs = robot_state_->getFeedbackTrajectoryPoint().feedback.digital_inputs;

    auto res = sequencer_.advance(world);
    // P5 execution annotation: publish the register snapshot and the last evaluated branch after
    // EVERY advance (success or fault) — the operator display must reflect what actually happened,
    // including the state a fault left behind.
    robot_state_->updateProgramExecution(sequencer_.registers(), sequencer_.lastBranchLine(),
                                         sequencer_.lastBranchTaken());
    if (res.isError()) {
        faultProgram(ToString(res.error()));
        return;
    }
    dispatchAction(res.value());
}

void RobotController::dispatchAction(const StepAction& action) {
    switch (action.kind) {
        case StepActionKind::Idle:
            // A blocking action is still in flight; nothing new to do this tick.
            break;

        case StepActionKind::PlanMotionChain: {
            // Build one continuous trajectory chain from the gathered motion steps. Per-waypoint
            // blending_radius selects exact stop (0) vs rounded corner (> 0) inside the planner.
            waiting_di_line_ = -1;
            robot_state_->updateProgramLine(action.executing_line);
            // Remember the dispatched chain so PAUSE/RESUME can re-plan its remaining waypoints
            // from the pause position (REQ-PAUSE-02).
            active_motion_chain_ = action.motion_chain;
            active_chain_first_line_ = action.executing_line;
            executing_motion_line_ = action.executing_line;
            if (!planChainSteps(action.motion_chain, action.executing_line)) {
                break; // planChainSteps faulted the program with an operator-visible reason
            }
            exec_state_ = ExecutionState::WaitingMotion;
            break;
        }

        case StepActionKind::SetOutput: {
            // Instant action with a hardware side effect: the sequencer DECIDED, the controller
            // ACTUATES through the HAL (mandate rule 6 - the DO write is visible right here). SIM
            // mode drives the simulator's loopback outputs; a backend without a DO channel refuses
            // with a typed error and the program FAULTS - never a silent no-op output.
            waiting_di_line_ = -1;
            robot_state_->updateProgramLine(action.executing_line);
            if (auto res = hw_manager_->setDigitalOutput(action.io_port, action.io_state); res.isError()) {
                RDT_LOG_ERROR(MODULE_NAME, "SET DO failed at step {}: DO[{}]={} ({}).",
                              action.executing_line, action.io_port,
                              action.io_state ? "HIGH" : "LOW", ToString(res.error()));
                faultProgram("SET DO failed: digital output unavailable on this backend");
                break;
            }
            RDT_LOG_INFO(MODULE_NAME, "SET DO at step {}: DO[{}]={}.",
                         action.executing_line, action.io_port, action.io_state ? "HIGH" : "LOW");
            // Stay Running: the next tick advances the sequencer past the (already consumed) step.
            break;
        }

        case StepActionKind::StartWaitTime:
            waiting_di_line_ = -1;
            robot_state_->updateProgramLine(action.executing_line);
            wait_duration_ = action.wait_duration;
            wait_start_time_ = std::chrono::steady_clock::now();
            exec_state_ = ExecutionState::WaitingTime;
            RDT_LOG_INFO(MODULE_NAME, "Waiting {} s (step {})...",
                wait_duration_.value(), action.executing_line);
            break;

        case StepActionKind::WaitForInput:
            robot_state_->updateProgramLine(action.executing_line);
            // Start the timeout only when a NEW WaitDI begins (its line differs from the one we are
            // already blocked on). Re-polls of the same wait keep the original start time, so the
            // timeout is accurate across re-polls and across loop iterations.
            if (action.executing_line != waiting_di_line_) {
                waiting_di_line_ = action.executing_line;
                wait_timeout_ = action.wait_timeout;
                wait_start_time_ = std::chrono::steady_clock::now();
                RDT_LOG_INFO(MODULE_NAME, "Waiting for DI port {} == {} (step {})...",
                    action.wait_condition.io_port, action.wait_condition.trigger_on_state,
                    action.executing_line);
            }
            exec_state_ = ExecutionState::WaitingDI;
            break;

        case StepActionKind::Break:
            // BREAK step (P4): an authored, immediate program stop — NOT a completion and NOT a
            // fault. The operator must see WHERE the program stopped itself.
            waiting_di_line_ = -1;
            robot_state_->updateProgramLine(action.executing_line);
            RDT_LOG_INFO(MODULE_NAME, "Program stopped by BREAK at step {}.", action.executing_line);
            robot_state_->updateSystemMessage(
                "Program stopped by BREAK at line " + std::to_string(action.executing_line), false);
            stopProgram();
            break;

        case StepActionKind::Finished:
            waiting_di_line_ = -1;
            RDT_LOG_INFO(MODULE_NAME, "Program finished successfully.");
            stopProgram();
            break;
    }
}

bool RobotController::planChainSteps(const std::vector<NetProtocol::ProgramStepStruct>& steps,
                                     std::int32_t first_line) {
    // Single conversion path from program Move steps to planner waypoints, shared by the initial
    // chain dispatch (dispatchAction) and the pause/resume re-plan (REQ-PAUSE-02) so both plan
    // through identical mapping rules.
    std::vector<TrajectoryPoint> chain;
    chain.reserve(steps.size());
    for (size_t i = 0; i < steps.size(); ++i) {
        const auto& move_step = steps[i];
        TrajectoryPoint tp;
        // Explicit motion-type mapping. The sequencer only admits MoveJ/MoveL/MoveC/MoveS
        // into a motion chain (isExecutableStepType + the chain-gathering loop), so no
        // default fallback is needed here.
        if (move_step.type == NetProtocol::StepType::MoveJ) {
            tp.header.motion_type = MotionType::JOINT;
        } else if (move_step.type == NetProtocol::StepType::MoveL) {
            tp.header.motion_type = MotionType::LIN;
        } else if (move_step.type == NetProtocol::StepType::MoveC) {
            tp.header.motion_type = MotionType::CIRC;
            tp.command.cartesian_via = move_step.cartesian_via;
        } else { // NetProtocol::StepType::MoveS
            // Consecutive SPLINE waypoints are gathered into one block by the planner
            // (TrajectoryPlanner::planMotionChain, REQ-SPL-01).
            tp.header.motion_type = MotionType::SPLINE;
        }
        tp.command.joint_target = move_step.joint_target;
        tp.command.cartesian_target = move_step.cartesian_target;
        // Validity derives from the motion TYPE at this wire boundary (same rule as the
        // planner's preview waypointFromStep): LIN/CIRC/SPLINE author a real pose; for
        // JOINT the joints are the command and FK derives the Cartesian at render time.
        tp.command.cartesian_valid = (tp.header.motion_type != MotionType::JOINT);
        tp.command.speed_ratio = move_step.speed_ratio / 100.0;
        tp.header.blending_radius = move_step.blending_radius_mm;
        tp.header.use_blending = move_step.blending_radius_mm.value() > 0.0;
        // Tag each waypoint with its program-step line so processMotionFeedback() advances the
        // executing line per waypoint as the robot physically reaches each one. On a resumed
        // chain first_line is the line of the first REMAINING step, so the tags keep matching
        // the authored program lines.
        tp.header.sequence_index =
            static_cast<uint32_t>(first_line + static_cast<std::int32_t>(i));
        chain.push_back(tp);
    }

    if (auto res = planner_->planMotionChain(chain); res.isError()) {
        RDT_LOG_ERROR(MODULE_NAME, "Planning failed for motion run at step {}. Error ID {}",
            first_line, static_cast<int>(res.error()));
        faultProgram("Motion planning failed");
        return false;
    }
    return true;
}

TrajectoryPoint RobotController::physicalHoldPoint() const {
    // Hold where the robot physically IS (feedback.joint_actual), not at the last streamed command
    // point (REQ-PAUSE-01). For backends that profile motion in firmware (execution_mode ==
    // InternalInterpolator, e.g. MKS) the streamed command leads or lags the physical axes by up
    // to a whole segment, so holding at the command made PAUSE/STOP keep driving the robot to the
    // stale command point instead of freezing it in place. On stream backends (SIM/UDP teleport)
    // actual == command within one cycle, so behaviour there is unchanged.
    TrajectoryPoint hold = robot_state_->getFeedbackTrajectoryPoint();
    hold.command.joint_target = hold.feedback.joint_actual;
    CartPose actual_pose{};
    if (solveFK(hold.command.joint_target, actual_pose)) {
        hold.command.cartesian_target = actual_pose;
        hold.command.cartesian_valid = true;
    } else {
        // Fail-closed: without a valid pose a later Cartesian plan from this state is refused by
        // the interpolator's start-pose gate instead of planning from a garbage pose. Joint-space
        // motion (and the zero-length JOINT hold itself) is unaffected.
        hold.command.cartesian_valid = false;
    }
    return hold;
}

void RobotController::faultProgram(const std::string& reason) {
    RDT_LOG_ERROR(MODULE_NAME, "Program fault: {}", reason);
    robot_state_->updateSystemMessage("Program error: " + reason, true);
    stopProgram();
    robot_state_->updateRobotMode(RobotMode::Error);
}

bool RobotController::isRobotReadyForMotion() const {
    if (robot_state_->isEStopActive()) return false;
    if (robot_state_->hasError()) return false;

    // ===================== TEMPORARY (MKS bring-up) =========================
    // The HAL-status readiness gate is lifted while the Motor Configurator link is being
    // brought up: a non-Ok HAL status (Error_CommunicationLost/DriveFault/Temperature, or
    // even Warning_SyncLost) would otherwise block startProgram/jog/homing entirely.
    // E-STOP and latched errors above STILL gate motion.
    // TODO(restore): re-enable the HAL-status check below before production.
    // if (!robot_state_->isSimulated()) {
    //     auto diag = robot_state_->getFeedbackTrajectoryPoint().diagnostics;
    //     if (diag.hal != HalStatus::Ok) return false;
    // }
    // =======================================================================
    return true;
}

bool RobotController::isTaskFinished() const {
    return planner_->isTaskFinished();
}



} // namespace RDT
// --- END OF FILE: HexaMotion/modules/controller/src/RobotController.cpp ---
