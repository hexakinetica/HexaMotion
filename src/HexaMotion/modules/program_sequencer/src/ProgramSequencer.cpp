// --- START OF FILE: HexaMotion/modules/program_sequencer/src/ProgramSequencer.cpp ---
/**
 * @file ProgramSequencer.cpp
 * @brief Implementation of the flat-program interpreter (see ProgramSequencer.h).
 */
#include "ProgramSequencer.h"

namespace RDT {

std::string ToString(SequencerError error) {
    switch (error) {
        case SequencerError::NotLoaded:
            return "NotLoaded: advance() was called before a program was loaded; nothing to execute.";
        case SequencerError::EmptyProgram:
            return "EmptyProgram: the program has no steps; there is nothing to run.";
        case SequencerError::DuplicateLabelId:
            return "DuplicateLabelId: two Label steps share the same id, so a jump target is ambiguous.";
        case SequencerError::UnresolvedJumpTarget:
            return "UnresolvedJumpTarget: a jump targets a label id that does not exist; the program is "
                   "not armed (this is exactly what prevents a default GOTO-to-step-0 infinite loop).";
        case SequencerError::UnsupportedStep:
            return "UnsupportedStep: the program contains a step type this controller cannot execute "
                   "(None, or a WAIT DI conditioned on a register); the program is not armed.";
        case SequencerError::RunawayLoop:
            return "RunawayLoop: the instant-step budget was exceeded within one tick (a loop with no "
                   "motion or wait to make progress); execution faulted to keep the control loop safe.";
        case SequencerError::InvalidRegisterIndex:
            return "InvalidRegisterIndex: a register step or condition addresses an index outside "
                   "R[0..15]; the program is not armed (executing it would read/write memory the "
                   "register file does not have).";
    }
    return "Unknown SequencerError.";
}

bool ProgramSequencer::isExecutableStepType(NetProtocol::StepType type) noexcept {
    switch (type) {
        // Executable in this phase.
        case NetProtocol::StepType::Comment:
        case NetProtocol::StepType::MoveJ:
        case NetProtocol::StepType::MoveL:
        case NetProtocol::StepType::MoveC:    // circular arc (MotionType::CIRC, docs/REQ_motion_circ.md)
        case NetProtocol::StepType::MoveS:    // spline point (MotionType::SPLINE, docs/REQ_motion_spline.md)
        case NetProtocol::StepType::WaitTime:
        case NetProtocol::StepType::WaitDI:   // blocking wait on a digital input (condition-based)
        case NetProtocol::StepType::SetDO:    // drive a digital output (P3; HAL DO channel exists)
        case NetProtocol::StepType::Label:
        case NetProtocol::StepType::JumpToLabel:
        case NetProtocol::StepType::ConditionalJump:
        case NetProtocol::StepType::SetVar:   // register write (P4)
        case NetProtocol::StepType::IncVar:   // register increment (P4)
        case NetProtocol::StepType::DecVar:   // register decrement (P4)
        case NetProtocol::StepType::Break:    // immediate program stop from code (P4)
            return true;
        // Malformed / uninitialized; fail-closed rather than silently skipped.
        case NetProtocol::StepType::None:
            return false;
    }
    return false;
}

bool ProgramSequencer::conditionSatisfied(const NetProtocol::Condition& condition,
                                          std::uint32_t digital_inputs) const noexcept {
    // Register compare (P4): R[register_index] `op` operand. The index is validated fail-closed at
    // load(); the range check here is a defensive guard with the same fail-safe polarity as the DI
    // path — an unverifiable condition never takes the branch.
    if (condition.source == NetProtocol::ConditionSource::Register) {
        if (condition.register_index >= kRegisterCount) {
            return false;
        }
        const std::int32_t value = registers_[condition.register_index];
        switch (condition.op) {
            case NetProtocol::CompareOp::Equal:       return value == condition.operand;
            case NetProtocol::CompareOp::NotEqual:    return value != condition.operand;
            case NetProtocol::CompareOp::GreaterThan: return value >  condition.operand;
            case NetProtocol::CompareOp::LessThan:    return value <  condition.operand;
        }
        return false;
    }

    // DI ports are 1-based; bit index = io_port - 1 (matches the HAL/UdpDriver DI packing). A port
    // outside the valid range cannot be verified, so the condition reads as NOT satisfied: a branch
    // is only taken on a positively confirmed input (fail-safe — never jump on an unverifiable port).
    if (condition.io_port < kMinDigitalInputPort || condition.io_port > kMaxDigitalInputPort) {
        return false;
    }
    const std::uint32_t bit = 1u << (condition.io_port - 1u);
    const bool level = (digital_inputs & bit) != 0u;
    return level == condition.trigger_on_state;
}

Result<void, SequencerError> ProgramSequencer::load(const NetProtocol::ProgramDataStruct& program) {
    loaded_ = false; // stay unarmed until every check passes (fail-closed)

    if (program.steps.empty()) {
        return SequencerError::EmptyProgram;
    }

    // Pass 1: reject any non-executable step type and build the label table (Label.id -> index),
    // rejecting duplicate label ids.
    std::unordered_map<std::uint32_t, std::size_t> labels;
    labels.reserve(program.steps.size());
    for (std::size_t i = 0; i < program.steps.size(); ++i) {
        const NetProtocol::ProgramStepStruct& step = program.steps[i];
        if (!isExecutableStepType(step.type)) {
            return SequencerError::UnsupportedStep;
        }
        if (step.type == NetProtocol::StepType::Label) {
            const auto [it, inserted] = labels.emplace(step.id, i);
            static_cast<void>(it);
            if (!inserted) {
                return SequencerError::DuplicateLabelId;
            }
        }
        // Register steps must address the fixed file R[0..kRegisterCount-1] (fail-closed).
        if ((step.type == NetProtocol::StepType::SetVar ||
             step.type == NetProtocol::StepType::IncVar ||
             step.type == NetProtocol::StepType::DecVar) &&
            step.reg_index >= kRegisterCount) {
            return SequencerError::InvalidRegisterIndex;
        }
        if (step.type == NetProtocol::StepType::ConditionalJump &&
            step.condition.source == NetProtocol::ConditionSource::Register &&
            step.condition.register_index >= kRegisterCount) {
            return SequencerError::InvalidRegisterIndex;
        }
        // A WaitDI waits on the WORLD; a register only changes via program steps, so a
        // register-source wait either passes instantly or hangs forever. Authoring nonsense —
        // refuse to arm rather than execute a guaranteed-wrong wait.
        if (step.type == NetProtocol::StepType::WaitDI &&
            step.condition.source != NetProtocol::ConditionSource::DigitalInput) {
            return SequencerError::UnsupportedStep;
        }
    }

    // Pass 2: every jump target must resolve to a known label. This is the fail-closed guard that
    // makes GOTO/IF safe: an unmapped jump_target_id (the default 0) is rejected here instead of
    // silently jumping to step 0.
    for (const NetProtocol::ProgramStepStruct& step : program.steps) {
        if (step.type == NetProtocol::StepType::JumpToLabel ||
            step.type == NetProtocol::StepType::ConditionalJump) {
            if (labels.find(step.jump_target_id) == labels.end()) {
                return SequencerError::UnresolvedJumpTarget;
            }
        }
    }

    program_ = program;
    label_index_ = std::move(labels);
    reset();
    loaded_ = true;
    return Result<void, SequencerError>::Success();
}

void ProgramSequencer::reset() {
    pc_ = 0;
    phase_ = Phase::Idle;
    pending_input_condition_ = NetProtocol::Condition{};
    pending_input_timeout_ = Seconds{0.0};
    current_line_ = -1;
    // Fresh RUN = deterministic counters: a program must never observe registers left over from a
    // previous run (or from a different program). The branch annotation resets with them.
    registers_.fill(0);
    last_branch_line_ = -1;
    last_branch_taken_ = false;
}

Result<StepAction, SequencerError> ProgramSequencer::advance(const WorldSample& world) {
    if (!loaded_) {
        return SequencerError::NotLoaded;
    }
    if (phase_ == Phase::Faulted) {
        // A fault (e.g. RunawayLoop) latches until reset()/load(); keep reporting it explicitly.
        return SequencerError::RunawayLoop;
    }
    // A motion/time action is still executing; do not re-dispatch until the controller confirms
    // completion via onActionCompleted().
    if (phase_ == Phase::AwaitingMotion || phase_ == Phase::AwaitingTime) {
        StepAction idle;
        idle.executing_line = current_line_;
        return idle; // kind == Idle
    }
    // A WaitDI is condition-based: poll the input ourselves each call. If it is satisfied the wait is
    // complete (pc_ already points past the WaitDI step) and we resume deciding below; otherwise keep
    // reporting the block so the controller can enforce the timeout.
    if (phase_ == Phase::AwaitingInput) {
        if (conditionSatisfied(pending_input_condition_, world.digital_inputs)) {
            phase_ = Phase::Idle;
        } else {
            StepAction waiting;
            waiting.kind = StepActionKind::WaitForInput;
            waiting.wait_condition = pending_input_condition_;
            waiting.wait_timeout = pending_input_timeout_;
            waiting.executing_line = current_line_;
            return waiting;
        }
    }
    if (phase_ == Phase::Finished) {
        StepAction done;
        done.kind = StepActionKind::Finished;
        done.executing_line = current_line_;
        return done;
    }

    // Consume instant steps until a blocking action or the end of the program, bounded by the
    // runaway-loop watchdog. Only instant steps (Label/Comment/GOTO/IF) iterate the loop; blocking
    // steps (motion/wait) and the end return immediately.
    std::uint32_t instant_steps = 0;
    while (true) {
        if (pc_ >= program_.steps.size()) {
            phase_ = Phase::Finished;
            StepAction done;
            done.kind = StepActionKind::Finished;
            done.executing_line = current_line_;
            return done;
        }

        const NetProtocol::ProgramStepStruct& step = program_.steps[pc_];
        switch (step.type) {
            case NetProtocol::StepType::MoveJ:
            case NetProtocol::StepType::MoveL:
            case NetProtocol::StepType::MoveC:
            case NetProtocol::StepType::MoveS: {
                // Gather the maximal contiguous run of motion steps so the controller can plan them as
                // one continuous chain (per-point blending decides exact-stop vs rounded corner; a
                // contiguous MoveS sub-run becomes one spline block inside the chain). A non-motion
                // step breaks the run and is handled on the next advance().
                StepAction action;
                action.kind = StepActionKind::PlanMotionChain;
                const std::size_t first = pc_;
                while (pc_ < program_.steps.size() &&
                       (program_.steps[pc_].type == NetProtocol::StepType::MoveJ ||
                        program_.steps[pc_].type == NetProtocol::StepType::MoveL ||
                        program_.steps[pc_].type == NetProtocol::StepType::MoveC ||
                        program_.steps[pc_].type == NetProtocol::StepType::MoveS)) {
                    action.motion_chain.push_back(program_.steps[pc_]);
                    ++pc_;
                }
                current_line_ = static_cast<std::int32_t>(first);
                action.executing_line = current_line_;
                phase_ = Phase::AwaitingMotion;
                return action;
            }

            case NetProtocol::StepType::WaitTime: {
                StepAction action;
                action.kind = StepActionKind::StartWaitTime;
                action.wait_duration = step.wait_duration_s;
                current_line_ = static_cast<std::int32_t>(pc_);
                action.executing_line = current_line_;
                ++pc_;
                phase_ = Phase::AwaitingTime;
                return action;
            }

            case NetProtocol::StepType::WaitDI: {
                // Block until the digital input matches. The sequencer owns the condition and polls it
                // in subsequent advance() calls; the controller enforces the timeout (sourced from the
                // step's wait_duration_s; 0 = wait indefinitely).
                pending_input_condition_ = step.condition;
                pending_input_timeout_ = step.wait_duration_s;
                current_line_ = static_cast<std::int32_t>(pc_);
                ++pc_;
                phase_ = Phase::AwaitingInput;

                StepAction action;
                action.kind = StepActionKind::WaitForInput;
                action.wait_condition = pending_input_condition_;
                action.wait_timeout = pending_input_timeout_;
                action.executing_line = current_line_;
                return action;
            }

            case NetProtocol::StepType::SetDO: {
                // Instant with a SIDE EFFECT: the controller drives the output through the HAL and
                // continues on the next advance() (pc already points past this step). A failed DO
                // write faults the program controller-side (fail-closed, never a silent no-op).
                StepAction action;
                action.kind = StepActionKind::SetOutput;
                action.io_port = step.io_port;
                action.io_state = step.io_state;
                current_line_ = static_cast<std::int32_t>(pc_);
                action.executing_line = current_line_;
                ++pc_;
                return action;
            }

            case NetProtocol::StepType::Comment:
            case NetProtocol::StepType::Label:
                // Instant, no side effect; fall through to the watchdog and continue.
                current_line_ = static_cast<std::int32_t>(pc_);
                ++pc_;
                break;

            case NetProtocol::StepType::SetVar:
            case NetProtocol::StepType::IncVar:
            case NetProtocol::StepType::DecVar: {
                // Instant, sequencer-INTERNAL state (unlike SetDO, nothing to actuate): mutate the
                // register and keep consuming instant steps under the watchdog. Index validated
                // fail-closed at load(); defend anyway.
                current_line_ = static_cast<std::int32_t>(pc_);
                if (step.reg_index >= kRegisterCount) {
                    phase_ = Phase::Faulted;
                    return SequencerError::InvalidRegisterIndex;
                }
                if (step.type == NetProtocol::StepType::SetVar) {
                    registers_[step.reg_index] = step.reg_value;
                } else if (step.type == NetProtocol::StepType::IncVar) {
                    registers_[step.reg_index] += 1;
                } else {
                    registers_[step.reg_index] -= 1;
                }
                ++pc_;
                break;
            }

            case NetProtocol::StepType::Break: {
                // Immediate program stop from code (boss decision: STOP semantics). Terminal for
                // this run; the controller stops execution and reports the line to the operator.
                current_line_ = static_cast<std::int32_t>(pc_);
                phase_ = Phase::Finished;
                StepAction action;
                action.kind = StepActionKind::Break;
                action.executing_line = current_line_;
                return action;
            }

            case NetProtocol::StepType::JumpToLabel: {
                current_line_ = static_cast<std::int32_t>(pc_);
                const auto it = label_index_.find(step.jump_target_id);
                if (it == label_index_.end()) {
                    // Guarded by load() validation; defend anyway rather than rely on it.
                    phase_ = Phase::Faulted;
                    return SequencerError::UnresolvedJumpTarget;
                }
                pc_ = it->second;
                break;
            }

            case NetProtocol::StepType::ConditionalJump: {
                current_line_ = static_cast<std::int32_t>(pc_);
                // P5 annotation: record which IF was evaluated last and its outcome; the status
                // loop carries it to the pendant so the operator sees WHY execution went where it
                // went (ProgramState.lastBranchLine/lastBranchTaken).
                last_branch_line_ = current_line_;
                if (conditionSatisfied(step.condition, world.digital_inputs)) {
                    last_branch_taken_ = true;
                    const auto it = label_index_.find(step.jump_target_id);
                    if (it == label_index_.end()) {
                        phase_ = Phase::Faulted;
                        return SequencerError::UnresolvedJumpTarget;
                    }
                    pc_ = it->second;
                } else {
                    last_branch_taken_ = false;
                    ++pc_;
                }
                break;
            }

            default:
                // Unreachable: load() fail-closes on non-executable types. Guard for safety.
                phase_ = Phase::Faulted;
                return SequencerError::UnsupportedStep;
        }

        if (++instant_steps > kMaxInstantStepsPerTick) {
            phase_ = Phase::Faulted;
            return SequencerError::RunawayLoop;
        }
    }
}

void ProgramSequencer::onActionCompleted() {
    // The controller reports that the in-flight blocking action (motion segment / wait) finished. pc_
    // already points past the completed step(s); resume deciding on the next advance().
    if (phase_ == Phase::AwaitingMotion || phase_ == Phase::AwaitingTime) {
        phase_ = Phase::Idle;
    }
}

} // namespace RDT
// --- END OF FILE: HexaMotion/modules/program_sequencer/src/ProgramSequencer.cpp ---
