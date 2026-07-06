// --- START OF FILE: HexaMotion/modules/program_sequencer/src/ProgramSequencer.h ---
/**
 * @file ProgramSequencer.h
 * @brief Controller-side executor of an authored flat robot program (loops, branches, waits).
 * @version 1.3 (P4: integer register file, SetVar/IncVar/DecVar, register-compare IF, BREAK)
 *
 * ProgramSequencer is a PURE interpreter over the flat program the pendant authored
 * (RDT::NetProtocol::ProgramDataStruct). It owns the program counter, the label table and a
 * runaway-loop watchdog, and DECIDES the next action (StepAction). It never touches the planner or
 * the HAL itself: the RobotController ACTUATES the returned action, which keeps the command -> hardware
 * data flow visible and lets this class be unit-tested with no Qt and no hardware.
 *
 * Design (see docs/program_sequencer.req.md and docs/REQ_program_sequencer.md):
 *  - FLAT control-flow model (labels + jumps); no structured-block compiler here (that stays on the
 *    pendant). The controller is the final arbiter and re-validates the program independently.
 *  - Fail-closed: an empty program, a duplicate label id, a jump to a non-existent label, or a step
 *    type this phase cannot execute is an error and the program is NOT armed (no silent skip).
 *  - Iterative dispatch (no recursion) with a per-tick instant-step budget, so a mis-authored
 *    "GOTO with no motion/wait" loop faults instead of overflowing the stack or hanging a tick.
 *  - Exception-free: fallible operations return RDT::Result<T, SequencerError>. No std::optional /
 *    std::variant / try-catch in the public surface (project mandate).
 *  - Completion of a TIMED/MOTION blocking action (motion segment reached / wait elapsed) is reported
 *    by the controller via onActionCompleted(); this class owns no wall clock. A WaitDI, by contrast,
 *    is condition-based: the sequencer owns the condition and self-completes when advance() sees the
 *    input satisfied, so its completion does NOT go through onActionCompleted().
 *
 * Staged scope: all four motion types (MoveJ/MoveL/MoveC/MoveS), WaitTime, Comment, Label,
 * JumpToLabel, ConditionalJump (DI or register-compare source), WaitDI (blocking DI wait, bounded by
 * a controller-enforced timeout), SetDO (P3: the controller actuates through the HAL), and the P4
 * register set: SetVar/IncVar/DecVar over a fixed integer register file (counter loops) plus Break
 * (immediate program stop from code). NOT executable (fail-closed at load): None (malformed) and a
 * WaitDI with a Register-source condition (registers only change via program steps, so such a wait
 * either passes instantly or hangs forever — it is authoring nonsense and refused).
 */
#ifndef RDT_PROGRAM_SEQUENCER_H
#define RDT_PROGRAM_SEQUENCER_H

#pragma once

#include "RdtProtocol.h"   // RDT::NetProtocol::ProgramDataStruct / ProgramStepStruct / StepType / Condition
#include "Units.h"         // RDT::Seconds
#include "result.h"        // RDT::Result

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace RDT {

/// @brief Typed reasons the sequencer cannot arm a program or must fault. No silent failures.
enum class SequencerError {
    NotLoaded,            ///< advance() was called before a program was successfully loaded.
    EmptyProgram,         ///< The program has no steps.
    DuplicateLabelId,     ///< Two Label steps share the same id (an ambiguous jump target).
    UnresolvedJumpTarget, ///< A JumpToLabel/ConditionalJump targets an id with no matching Label.
    UnsupportedStep,      ///< A step type is not executable (None, or a Register-source WaitDI).
    RunawayLoop,          ///< The per-tick instant-step budget was exceeded (loop with no progress).
    InvalidRegisterIndex  ///< A register step/condition addresses an index outside R[0..kRegisterCount-1].
};

/// @brief Human-readable diagnostic for a SequencerError (what happened / why it is a problem).
[[nodiscard]] std::string ToString(SequencerError error);

/// @brief What the controller must do next. The sequencer decides; it never actuates.
enum class StepActionKind {
    Idle,            ///< Nothing new this tick (a motion/time action is in flight, or execution is done).
    PlanMotionChain, ///< Plan `motion_chain` (the maximal contiguous run of motion steps).
    StartWaitTime,   ///< Block for `wait_duration` (the controller owns the timer).
    WaitForInput,    ///< Block until `wait_condition` is met; the controller enforces `wait_timeout`.
    SetOutput,       ///< Drive digital output `io_port` to `io_state` (instant; the controller actuates).
    Break,           ///< BREAK step reached: stop the program NOW (boss decision: STOP-from-code
                     ///< semantics). The controller stops execution and reports the line; distinct
                     ///< from Finished so the operator sees "stopped by BREAK", not "completed".
    Finished         ///< The program completed successfully.
};

/// @brief One decision returned by advance(). Only the fields relevant to `kind` are populated.
struct StepAction {
    StepActionKind kind = StepActionKind::Idle;
    /// PlanMotionChain: the contiguous MoveJ/MoveL run to plan as one continuous chain.
    std::vector<NetProtocol::ProgramStepStruct> motion_chain;
    /// StartWaitTime: how long to wait.
    Seconds wait_duration{0.0};
    /// WaitForInput: the digital-input condition the sequencer is blocked on. The sequencer polls it
    /// itself on each advance() (it owns the condition); the controller only enforces the timeout.
    NetProtocol::Condition wait_condition{};
    /// WaitForInput: the maximum time to wait before the controller faults the wait. Sourced from the
    /// step's wait_duration_s; 0 means wait indefinitely (by design). Proposed policy pending the boss
    /// decision on WaitDI timeouts (REQ_program_sequencer.md §11).
    Seconds wait_timeout{0.0};
    /// SetOutput: the digital output to drive (1-based port) and the level. The sequencer only
    /// DECIDES; the controller actuates through the HAL (data flow stays visible, mandate rule 6).
    std::uint16_t io_port = 0;
    bool io_state = false;
    /// Index of the step that produced this action (display only: the executing-line highlight).
    std::int32_t executing_line = -1;
};

/**
 * @brief The slice of the circulating TrajectoryPoint the controller distils for the sequencer.
 *
 * P1 needs only the digital-input bitmask, and only for an instant ConditionalJump (IF-on-DI) branch.
 * It grows in later phases (registers for counter branches); motion/time completion is NOT carried
 * here — it is signalled by onActionCompleted() so this class stays clock-free and testable.
 */
struct WorldSample {
    std::uint32_t digital_inputs = 0; ///< Latest DI bitmask (bit i == DI(i+1)); rides the feedback loop.
};

/**
 * @class ProgramSequencer
 * @brief Pure, testable interpreter of the flat program; owned and driven by RobotController.
 */
class ProgramSequencer {
public:
    /// Runaway-loop guard: max instant steps (Label/Comment/GOTO/IF) consumed within one advance()
    /// before the sequencer faults. Generous for legitimate straight-line runs; only a true
    /// no-progress loop (jumps with no motion/wait in between) can reach it.
    static constexpr std::uint32_t kMaxInstantStepsPerTick = 10000;

    /// DI ports are 1-based (DI1..DIkMaxDigitalInputPort); bit index = io_port - 1 (matches the
    /// HAL/UdpDriver DI packing). A condition on a port outside this range is treated as unsatisfied.
    static constexpr std::uint16_t kMinDigitalInputPort = 1;
    static constexpr std::uint16_t kMaxDigitalInputPort = 32;

    /// Integer register file size (P4, boss decision: 16). Registers are 0-based R[0..15], cleared
    /// on every fresh RUN (reset()) — a program always starts with deterministic counters. Aliases
    /// the wire constant: the P5 status annotation carries the same file on ProgramState.
    static constexpr std::uint16_t kRegisterCount =
        static_cast<std::uint16_t>(NetProtocol::kProgramRegisterCount);

    ProgramSequencer() = default;

    ProgramSequencer(const ProgramSequencer&) = delete;
    ProgramSequencer& operator=(const ProgramSequencer&) = delete;

    /**
     * @brief Validate the program, build the label table and arm execution from step 0.
     * @return Success, or the first blocking SequencerError (the program is left unarmed on error).
     * @details Fail-closed: rejects an empty program, duplicate label ids, any jump to a
     *          non-existent label, and any step type not executable in this phase. Independent of the
     *          pendant's own validation (the controller is the final arbiter).
     */
    Result<void, SequencerError> load(const NetProtocol::ProgramDataStruct& program);

    /// @brief Re-arm the loaded program from step 0 (fresh RUN). Keeps the program/label table.
    void reset();

    /// @brief True once a program has been successfully loaded.
    [[nodiscard]] bool isLoaded() const noexcept { return loaded_; }

    /**
     * @brief Decide the next action.
     * @param world The distilled controller snapshot (DI for IF-on-DI branches).
     * @return The next StepAction, or a SequencerError if the sequencer faults (e.g. RunawayLoop).
     * @details Consumes instant steps (Label/Comment/GOTO/IF) up to the watchdog budget and stops at
     *          the first blocking action (PlanMotionChain/StartWaitTime/WaitForInput) or Finished.
     *          While a motion/time action is in flight it returns Idle until onActionCompleted() is
     *          called. While blocked on a WaitDI it re-checks `world.digital_inputs` each call and
     *          keeps returning WaitForInput until the condition is met (then it advances itself).
     */
    Result<StepAction, SequencerError> advance(const WorldSample& world);

    /// @brief Controller callback: the in-flight motion/time action has completed. (A WaitDI completes
    /// itself inside advance() when the input is satisfied, so it does not use this callback.)
    void onActionCompleted();

    /// @brief The step index to highlight in the UI (display only). -1 before the first step.
    [[nodiscard]] std::int32_t currentLine() const noexcept { return current_line_; }

    /// @brief Read-only view of the register file (tests; the P5 execution annotation).
    [[nodiscard]] const std::array<std::int32_t, kRegisterCount>& registers() const noexcept {
        return registers_;
    }

    /// @brief Step index of the last evaluated ConditionalJump this run (-1 = none yet) and its
    /// outcome — the P5 execution annotation ("branch taken") riding ProgramState.
    [[nodiscard]] std::int32_t lastBranchLine() const noexcept { return last_branch_line_; }
    [[nodiscard]] bool lastBranchTaken() const noexcept { return last_branch_taken_; }

private:
    /// Internal execution phase (distinct from the controller's ExecutionState; this owns no timing).
    enum class Phase { Idle, AwaitingMotion, AwaitingTime, AwaitingInput, Finished, Faulted };

    /// @brief True if the step type can be executed in this phase (used for fail-closed load()).
    [[nodiscard]] static bool isExecutableStepType(NetProtocol::StepType type) noexcept;

    /// @brief Evaluate a ConditionalJump condition against the DI bitmask (fail-safe on bad port).
    [[nodiscard]] bool conditionSatisfied(const NetProtocol::Condition& condition,
                                          std::uint32_t digital_inputs) const noexcept;

    NetProtocol::ProgramDataStruct program_{};                 ///< The armed program (owned copy).
    std::unordered_map<std::uint32_t, std::size_t> label_index_; ///< Label.id -> step index.
    std::array<std::int32_t, kRegisterCount> registers_{};     ///< Integer register file (counters).
    std::int32_t last_branch_line_ = -1;                       ///< Last evaluated IF (P5 annotation).
    bool last_branch_taken_ = false;                           ///< Whether that IF jumped.
    std::size_t pc_ = 0;                                       ///< Program counter.
    Phase phase_ = Phase::Idle;
    NetProtocol::Condition pending_input_condition_{};         ///< The DI condition an active WaitDI blocks on.
    Seconds pending_input_timeout_{0.0};                       ///< Timeout re-reported to the controller each poll.
    std::int32_t current_line_ = -1;
    bool loaded_ = false;
};

} // namespace RDT

#endif // RDT_PROGRAM_SEQUENCER_H
// --- END OF FILE: HexaMotion/modules/program_sequencer/src/ProgramSequencer.h ---
