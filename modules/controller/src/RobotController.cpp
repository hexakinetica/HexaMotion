// RobotController.cpp
#include "RobotController.h"
#include <stdexcept>
#include <iostream>
namespace RDT {

using namespace RDT::literals;

RobotController::RobotController(std::shared_ptr<HardwareManager> hw_manager,
                                 std::shared_ptr<MotionManager> motion_manager,
                                 std::shared_ptr<TrajectoryPlanner> planner,
                                 std::shared_ptr<KinematicSolver> solver,
                                 std::shared_ptr<RobotState> robot_state)
    : hw_manager_(hw_manager),
      motion_manager_(motion_manager),
      planner_(planner),
      solver_(solver),
      robot_state_(robot_state)
{
    if (!hw_manager_ || !motion_manager_ || !planner_ || !solver_ || !robot_state_) {
        RDT_LOG_CRITICAL(MODULE_NAME, "Dependencies cannot be null.");
        throw std::invalid_argument("RobotController dependencies cannot be null.");
    }
}

RobotController::~RobotController() = default;

bool RobotController::initialize() {
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
    if (!solver_->solveFK(initial_tp.command.joint_target, initial_tp.command.cartesian_target)) {
        RDT_LOG_ERROR(MODULE_NAME, "Initial FK failed.");
        // Continue, but warn. Planner might correct this on first move.
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
}

void RobotController::processMotionFeedback() {
    TrajectoryPoint feedback;
    // Drain the feedback queue to get the latest state and process all events
    while (motion_manager_->dequeueFeedback(feedback)) {
        
        // Update RobotState source of truth
        robot_state_->updateFeedbackTrajectoryPoint(feedback);

        // Check Diagnostics / Errors
        if (!feedback.diagnostics.isHealthy()) {
            if (feedback.diagnostics.safety != SafetyStatus::Ok) {
                RDT_LOG_ERROR(MODULE_NAME, "Safety Error Detected! Code: {}", static_cast<int>(feedback.diagnostics.safety));
                robot_state_->updateSystemMessage("Safety Error: Motion Stopped", true);
                robot_state_->updateRobotMode(RobotMode::Error);
                stopProgram();
            } else if (feedback.diagnostics.hal != HalStatus::Ok && feedback.diagnostics.hal != HalStatus::NotConnected) {
                 RDT_LOG_ERROR(MODULE_NAME, "HAL Error Detected! Code: {}", static_cast<int>(feedback.diagnostics.hal));
                 robot_state_->updateSystemMessage("Hardware Error", true);
                 robot_state_->updateRobotMode(RobotMode::Error);
                 stopProgram();
            }
        }
    }
}

void RobotController::processNetworkCommands() {
    auto cmd = robot_state_->getLastReceivedCommand();

    // 1. Emergency Stop
    if (cmd.setEStop != last_estop_req_) {
        handleEmergencyStop(cmd.setEStop);
        last_estop_req_ = cmd.setEStop;
    }

    // If E-Stop is active, ignore other commands
    if (robot_state_->isEStopActive()) return;

    // 2. Clear Error
    if (cmd.clearError && robot_state_->hasError()) {
        handleClearError();
    }

    // If in Error, ignore other commands
    if (robot_state_->hasError()) return;

    // 3. Mode Switch (Sim <-> Real)
    if (cmd.enableRealMode != last_real_mode_req_) {
        handleModeSwitch(cmd.enableRealMode);
        last_real_mode_req_ = cmd.enableRealMode;
    }

    // 4. Program Control
    if (cmd.programCommand != last_program_cmd_) {
        handleProgramCommand(cmd.programCommand);
        last_program_cmd_ = cmd.programCommand;
    }

    // 5. Jogging
    if (cmd.jogRequestId != last_processed_jog_id_) {
        handleJogRequest(cmd);
        last_processed_jog_id_ = cmd.jogRequestId;
    }
    
    // 6. Update Overrides
    if (cmd.speedOverride >= 0.0) {
        robot_state_->updateSpeedRatio(cmd.speedOverride);
    }
}

void RobotController::handleEmergencyStop(bool active) {
    if (active) {
        RDT_LOG_CRITICAL(MODULE_NAME, "E-STOP REQUESTED!");
        motion_manager_->emergencyStop();
        robot_state_->updateEStopState(true);
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
    current_tp.header.motion_type = MotionType::JOINT;
    current_tp.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD;
    current_tp.command.joint_target = current_tp.feedback.joint_actual;
    
    (void)planner_->overrideTrajectory(current_tp); // Reset NRT planner
    
    robot_state_->updateSystemMessage("Ready", false);
    robot_state_->updateRobotMode(RobotMode::Idle);
}

void RobotController::handleModeSwitch(bool enable_real_mode) {
    HalMode target_mode = enable_real_mode ? HalMode::Realtime : HalMode::Simulation;
    
    if (enable_real_mode && hw_manager_->getMode() == HalMode::Simulation) {
        // Try to sync sim to real before switching
        hw_manager_->syncSimulationToReal();
    }

    auto res = hw_manager_->setMode(target_mode);
    if (res.isSuccess()) {
        robot_state_->updateSimulatedState(target_mode == HalMode::Simulation);
        RDT_LOG_INFO(MODULE_NAME, "Mode switched to {}", enable_real_mode ? "REAL" : "SIM");
    } else {
        RDT_LOG_ERROR(MODULE_NAME, "Mode switch failed: {}", ToString(res.error()));
        robot_state_->updateSystemMessage("Mode Switch Failed: Check Sync", true);
    }
}

void RobotController::handleJogRequest(const NetProtocol::ControlState& cmd) {
    if (!isRobotReadyForMotion()) return;
    
    if (exec_state_ != ExecutionState::Stopped) {
        RDT_LOG_WARN(MODULE_NAME, "Cannot jog while program is running.");
        return;
    }

    // Apply Jog Logic
    if (cmd.jogFrame == NetProtocol::JogFrame::JOINT) {
        if (cmd.jogAxis >= 0 && cmd.jogAxis < static_cast<int>(ROBOT_AXES_COUNT)) {
            // MATCH Program MoveJ style EXACTLY
            TrajectoryPoint tp;
            tp.header.motion_type = MotionType::JOINT;
            tp.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD;
            tp.command.speed_ratio = 0.5; // Same as MoveJ test (50%)
            tp.command.acceleration_ratio = 0.5;

            // Use the current physical position as the foundation for the command
            auto current_fb = robot_state_->getFeedbackTrajectoryPoint();
            tp.command.joint_target = current_fb.feedback.joint_actual;
            
            // Increment the requested axis
            auto axis_id = static_cast<AxisId>(cmd.jogAxis);
            tp.command.joint_target[axis_id].position += Degrees(cmd.jogIncrement);
            
            // IMPORTANT: Match MoveJ by NOT clearing the planner state manually
            // and using addTargetWaypoint instead of override
            auto plan_res = planner_->addTargetWaypoint(tp);

            if (plan_res.isError()) {
                RDT_LOG_WARN(MODULE_NAME, "Jog planning failed. Error: {}", static_cast<int>(plan_res.error()));
            } else {
                RDT_LOG_INFO(MODULE_NAME, "Jog planned successfully to axis {}: {}", 
                             cmd.jogAxis, tp.command.joint_target[axis_id].position.value());
            }
        }
    } else {
        RDT_LOG_WARN(MODULE_NAME, "Cartesian Jogging not fully implemented in this iteration.");
    }
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

void RobotController::startProgram() {
    if (!isRobotReadyForMotion()) return;
    
    if (exec_state_ == ExecutionState::Paused) {
        exec_state_ = ExecutionState::Running;
        RDT_LOG_INFO(MODULE_NAME, "Resuming program.");
    } else {
        // Start from beginning
        const auto& prog = robot_state_->getLoadedProgram();
        if (prog.steps.empty()) {
            RDT_LOG_WARN(MODULE_NAME, "Cannot start empty program.");
            return;
        }
        current_program_step_index_ = 0;
        exec_state_ = ExecutionState::Running;
        robot_state_->updateRobotMode(RobotMode::Running);
        RDT_LOG_INFO(MODULE_NAME, "Starting program: {}", prog.name);
    }
}

void RobotController::stopProgram() {
    if (exec_state_ != ExecutionState::Stopped) {
        exec_state_ = ExecutionState::Stopped;
        
        auto current_tp = robot_state_->getFeedbackTrajectoryPoint();
        current_tp.header.motion_type = MotionType::JOINT;
        current_tp.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD;
        current_tp.command.joint_target = current_tp.feedback.joint_actual;
        
        (void)planner_->overrideTrajectory(current_tp); // Stop motion
        
        robot_state_->updateRobotMode(RobotMode::Idle);
        RDT_LOG_INFO(MODULE_NAME, "Program stopped.");
    }
}

void RobotController::pauseProgram() {
    if (exec_state_ == ExecutionState::Running) {
        exec_state_ = ExecutionState::Paused;
        
        auto current_tp = robot_state_->getFeedbackTrajectoryPoint();
        current_tp.header.motion_type = MotionType::JOINT;
        current_tp.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD;
        current_tp.command.joint_target = current_tp.feedback.joint_actual;
        
        (void)planner_->overrideTrajectory(current_tp);
        
        robot_state_->updateRobotMode(RobotMode::Paused);
        RDT_LOG_INFO(MODULE_NAME, "Program paused.");
    }
}

void RobotController::handleProgramExecution() {
    // Update planner regardless of program state (to feed buffer)
    planner_->update();

    if (exec_state_ != ExecutionState::Running && exec_state_ != ExecutionState::WaitingTime) return;

    // If waiting for time
    if (exec_state_ == ExecutionState::WaitingTime) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::duration<double>>(now - wait_start_time_).count() >= wait_duration_.value()) {
            exec_state_ = ExecutionState::Running; // Wait finished
            executeNextStep();
        }
        return;
    }

    // If running, check if previous motion is done
    if (planner_->isTaskFinished()) {
        executeNextStep();
    }
}

void RobotController::executeNextStep() {
    const auto& prog = robot_state_->getLoadedProgram();
    if (current_program_step_index_ >= prog.steps.size()) {
        RDT_LOG_INFO(MODULE_NAME, "Program finished successfully.");
        stopProgram();
        return;
    }

    const auto& step = prog.steps[current_program_step_index_];
    robot_state_->updateProgramLine(static_cast<int>(current_program_step_index_));
    current_program_step_index_++;

    switch (step.type) {
        case NetProtocol::StepType::MoveJ:
        case NetProtocol::StepType::MoveL: {
            TrajectoryPoint tp;
            // Build TP from step data
            tp.header.motion_type = (step.type == NetProtocol::StepType::MoveJ) ? MotionType::JOINT : MotionType::LIN;
            tp.header.data_type = (step.type == NetProtocol::StepType::MoveJ) ? WaypointDataType::JOINT_DOMINANT_CMD : WaypointDataType::CARTESIAN_DOMINANT_CMD;
            tp.header.use_blending = (step.blending_radius_mm.value() > 0.0); // Simple check
            
            tp.header.tool = ToolFrame(); 
            tp.header.base = BaseFrame();

            tp.command.joint_target = step.joint_target;
            tp.command.cartesian_target = step.cartesian_target;
            tp.command.speed_ratio = step.speed_ratio / 100.0; // Convert % to ratio

            auto res = planner_->addTargetWaypoint(tp);
            if (res.isError()) {
                RDT_LOG_ERROR(MODULE_NAME, "Planning failed for step {}. Error: ID {}", current_program_step_index_, static_cast<int>(res.error()));
                robot_state_->updateSystemMessage("Planning Failed", true);
                stopProgram();
                robot_state_->updateRobotMode(RobotMode::Error);
            }
            break;
        }
        case NetProtocol::StepType::WaitTime:
            handleWaitStep(step);
            break;
        case NetProtocol::StepType::Comment:
            // Just skip
            executeNextStep(); 
            break;
        default:
            RDT_LOG_WARN(MODULE_NAME, "Unsupported step type: {}", static_cast<int>(step.type));
            // Skip for now
            executeNextStep();
            break;
    }
}

void RobotController::handleWaitStep(const NetProtocol::ProgramStepStruct& step) {
    wait_duration_ = step.wait_duration_s;
    wait_start_time_ = std::chrono::steady_clock::now();
    exec_state_ = ExecutionState::WaitingTime;
    RDT_LOG_INFO(MODULE_NAME, "Waiting for {} seconds...", wait_duration_.value());
}

bool RobotController::isRobotReadyForMotion() const {
    if (robot_state_->isEStopActive()) return false;
    if (robot_state_->hasError()) return false;
    return true;
}

} // namespace RDT
