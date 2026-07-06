// --- START OF FILE: HexaMotion/modules/program_sequencer/tests/gtest_program_sequencer_main.cpp ---
/**
 * @file gtest_program_sequencer_main.cpp
 * @brief Unit tests for the standalone ProgramSequencer (P1). Pure logic, no hardware, no clock.
 *
 * The sequencer decides; the controller actuates. These tests drive the decision surface directly:
 * load() validation (fail-closed), instant-step dispatch, motion-chain gathering, WaitTime,
 * unconditional/conditional jumps (loops and branches), the runaway-loop watchdog and reset().
 */
#include "gtest/gtest.h"

#include "ProgramSequencer.h"
#include "RdtProtocol.h"
#include "Units.h"

using namespace RDT;
using namespace RDT::NetProtocol;

// --- Step factories (build a ProgramStepStruct for a specific step type) ---
namespace {

ProgramStepStruct makeMotion(std::uint32_t id, StepType type) {
    ProgramStepStruct s;
    s.id = id;
    s.type = type; // MoveJ or MoveL
    s.speed_ratio = 100.0;
    return s;
}

ProgramStepStruct makeWait(std::uint32_t id, double seconds) {
    ProgramStepStruct s;
    s.id = id;
    s.type = StepType::WaitTime;
    s.wait_duration_s = Seconds{seconds};
    return s;
}

ProgramStepStruct makeComment(std::uint32_t id) {
    ProgramStepStruct s;
    s.id = id;
    s.type = StepType::Comment;
    return s;
}

ProgramStepStruct makeLabel(std::uint32_t id) {
    ProgramStepStruct s;
    s.id = id;
    s.type = StepType::Label;
    return s;
}

ProgramStepStruct makeGoto(std::uint32_t id, std::uint32_t target_label_id) {
    ProgramStepStruct s;
    s.id = id;
    s.type = StepType::JumpToLabel;
    s.jump_target_id = target_label_id;
    return s;
}

ProgramStepStruct makeIf(std::uint32_t id, std::uint16_t io_port, bool trigger_on_state,
                         std::uint32_t target_label_id) {
    ProgramStepStruct s;
    s.id = id;
    s.type = StepType::ConditionalJump;
    s.condition.io_port = io_port;
    s.condition.trigger_on_state = trigger_on_state;
    s.jump_target_id = target_label_id;
    return s;
}

ProgramStepStruct makeSetDo(std::uint32_t id, std::uint16_t io_port, bool io_state) {
    ProgramStepStruct s;
    s.id = id;
    s.type = StepType::SetDO;
    s.io_port = io_port;
    s.io_state = io_state;
    return s;
}

ProgramStepStruct makeWaitDI(std::uint32_t id, std::uint16_t io_port, bool trigger_on_state,
                             double timeout_s) {
    ProgramStepStruct s;
    s.id = id;
    s.type = StepType::WaitDI;
    s.condition.io_port = io_port;
    s.condition.trigger_on_state = trigger_on_state;
    s.wait_duration_s = Seconds{timeout_s}; // reused as the wait timeout (0 = infinite)
    return s;
}

ProgramStepStruct makeRaw(std::uint32_t id, StepType type) {
    ProgramStepStruct s;
    s.id = id;
    s.type = type;
    return s;
}

ProgramDataStruct makeProgram(std::vector<ProgramStepStruct> steps) {
    ProgramDataStruct p;
    p.name = "test_program";
    p.steps = std::move(steps);
    return p;
}

// DI bitmask helper: port is 1-based (DI1 -> bit 0), matching the sequencer's convention.
std::uint32_t diBit(std::uint16_t port) { return 1u << (port - 1u); }

// --- P4 register / BREAK helpers -------------------------------------------

ProgramStepStruct makeSetVar(std::uint32_t id, std::uint16_t reg_index, std::int32_t value) {
    ProgramStepStruct s;
    s.id = id;
    s.type = StepType::SetVar;
    s.reg_index = reg_index;
    s.reg_value = value;
    return s;
}

ProgramStepStruct makeIncVar(std::uint32_t id, std::uint16_t reg_index) {
    ProgramStepStruct s;
    s.id = id;
    s.type = StepType::IncVar;
    s.reg_index = reg_index;
    return s;
}

ProgramStepStruct makeDecVar(std::uint32_t id, std::uint16_t reg_index) {
    ProgramStepStruct s;
    s.id = id;
    s.type = StepType::DecVar;
    s.reg_index = reg_index;
    return s;
}

ProgramStepStruct makeIfReg(std::uint32_t id, std::uint16_t reg_index, CompareOp op,
                            std::int32_t operand, std::uint32_t target_label_id) {
    ProgramStepStruct s;
    s.id = id;
    s.type = StepType::ConditionalJump;
    s.condition.source = ConditionSource::Register;
    s.condition.register_index = reg_index;
    s.condition.op = op;
    s.condition.operand = operand;
    s.jump_target_id = target_label_id;
    return s;
}

ProgramStepStruct makeBreak(std::uint32_t id) {
    ProgramStepStruct s;
    s.id = id;
    s.type = StepType::Break;
    return s;
}

} // namespace

// ============================================================================
// load() validation — fail-closed
// ============================================================================

TEST(ProgramSequencer, LoadRejectsEmptyProgram) {
    ProgramSequencer seq;
    const auto r = seq.load(makeProgram({}));
    ASSERT_TRUE(r.isError());
    EXPECT_EQ(r.error(), SequencerError::EmptyProgram);
    EXPECT_FALSE(seq.isLoaded());
}

TEST(ProgramSequencer, LoadRejectsUnresolvedJumpTarget) {
    // GOTO to a label id that does not exist must be rejected (prevents the default GOTO-to-0 loop).
    ProgramSequencer seq;
    const auto r = seq.load(makeProgram({makeMotion(0, StepType::MoveL), makeGoto(1, 99)}));
    ASSERT_TRUE(r.isError());
    EXPECT_EQ(r.error(), SequencerError::UnresolvedJumpTarget);
    EXPECT_FALSE(seq.isLoaded());
}

TEST(ProgramSequencer, LoadRejectsDuplicateLabelId) {
    ProgramSequencer seq;
    const auto r = seq.load(makeProgram({makeLabel(5), makeLabel(5), makeMotion(2, StepType::MoveL)}));
    ASSERT_TRUE(r.isError());
    EXPECT_EQ(r.error(), SequencerError::DuplicateLabelId);
}

TEST(ProgramSequencer, LoadRejectsUnsupportedStepTypes) {
    // Everything authored is executable as of P3 (SetDO included); only None stays fail-closed.
    ProgramSequencer seq;
    const auto r = seq.load(makeProgram({makeMotion(0, StepType::MoveL), makeRaw(1, StepType::None)}));
    ASSERT_TRUE(r.isError());
    EXPECT_EQ(r.error(), SequencerError::UnsupportedStep);
}

TEST(ProgramSequencer, SetDoEmitsOutputActionAndContinues) {
    // P3: SET DO is an instant action WITH a side effect - the sequencer emits SetOutput (port +
    // level) for the controller to actuate, and execution continues on the next advance().
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({makeSetDo(0, /*port*/ 5, /*state*/ true),
                                      makeMotion(1, StepType::MoveJ)})).isSuccess());
    const auto first = seq.advance(WorldSample{});
    ASSERT_TRUE(first.isSuccess());
    EXPECT_EQ(first.value().kind, StepActionKind::SetOutput);
    EXPECT_EQ(first.value().io_port, 5);
    EXPECT_TRUE(first.value().io_state);
    EXPECT_EQ(first.value().executing_line, 0);

    const auto second = seq.advance(WorldSample{});
    ASSERT_TRUE(second.isSuccess());
    EXPECT_EQ(second.value().kind, StepActionKind::PlanMotionChain);
}

TEST(ProgramSequencer, MoveCIsExecutableAndJoinsMotionChain) {
    // MoveC (CIRC) is a first-class motion step (docs/REQ_motion_circ.md): it must pass load
    // validation and be gathered into one continuous chain together with MoveJ/MoveL.
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({
        makeMotion(0, StepType::MoveJ),
        makeMotion(1, StepType::MoveC),
        makeMotion(2, StepType::MoveL),
        makeWait(3, 0.5),
    })).isSuccess());

    WorldSample world{};
    const auto a = seq.advance(world);
    ASSERT_TRUE(a.isSuccess());
    ASSERT_EQ(a.value().kind, StepActionKind::PlanMotionChain);
    ASSERT_EQ(a.value().motion_chain.size(), 3u);
    EXPECT_EQ(a.value().motion_chain[1].type, StepType::MoveC);
}

TEST(ProgramSequencer, MoveSIsExecutableAndJoinsMotionChain) {
    // MoveS (spline point) is a first-class motion step (docs/REQ_motion_spline.md): a contiguous
    // MoveS run must pass load validation and join the motion chain with the other move types —
    // the planner turns the run into ONE spline block (REQ-SPL-01).
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({
        makeMotion(0, StepType::MoveL),
        makeMotion(1, StepType::MoveS),
        makeMotion(2, StepType::MoveS),
        makeMotion(3, StepType::MoveS),
        makeWait(4, 0.5),
    })).isSuccess());

    WorldSample world{};
    const auto a = seq.advance(world);
    ASSERT_TRUE(a.isSuccess());
    ASSERT_EQ(a.value().kind, StepActionKind::PlanMotionChain);
    ASSERT_EQ(a.value().motion_chain.size(), 4u);
    EXPECT_EQ(a.value().motion_chain[1].type, StepType::MoveS);
    EXPECT_EQ(a.value().motion_chain[3].type, StepType::MoveS);
}

TEST(ProgramSequencer, LoadAcceptsWellFormedProgram) {
    ProgramSequencer seq;
    const auto r = seq.load(makeProgram({makeLabel(1), makeMotion(1, StepType::MoveL), makeGoto(2, 1)}));
    ASSERT_TRUE(r.isSuccess());
    EXPECT_TRUE(seq.isLoaded());
}

// ============================================================================
// advance() basics
// ============================================================================

TEST(ProgramSequencer, AdvanceBeforeLoadFails) {
    ProgramSequencer seq;
    const auto a = seq.advance(WorldSample{});
    ASSERT_TRUE(a.isError());
    EXPECT_EQ(a.error(), SequencerError::NotLoaded);
}

TEST(ProgramSequencer, SingleMotionThenFinished) {
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({makeMotion(0, StepType::MoveL)})).isSuccess());

    auto a = seq.advance(WorldSample{});
    ASSERT_TRUE(a.isSuccess());
    EXPECT_EQ(a.value().kind, StepActionKind::PlanMotionChain);
    EXPECT_EQ(a.value().motion_chain.size(), 1u);
    EXPECT_EQ(a.value().executing_line, 0);
    EXPECT_EQ(seq.currentLine(), 0);

    // While the motion is in flight, advance() yields Idle (no re-dispatch) until completion.
    EXPECT_EQ(seq.advance(WorldSample{}).value().kind, StepActionKind::Idle);

    seq.onActionCompleted();
    EXPECT_EQ(seq.advance(WorldSample{}).value().kind, StepActionKind::Finished);
}

TEST(ProgramSequencer, MotionRunIsChainedAndWaitBreaksIt) {
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({
        makeMotion(0, StepType::MoveJ),
        makeMotion(1, StepType::MoveL),
        makeMotion(2, StepType::MoveJ),
        makeWait(3, 1.5),
        makeMotion(4, StepType::MoveL),
    })).isSuccess());

    // The three contiguous motion steps are gathered into one chain.
    auto a1 = seq.advance(WorldSample{});
    ASSERT_TRUE(a1.isSuccess());
    EXPECT_EQ(a1.value().kind, StepActionKind::PlanMotionChain);
    EXPECT_EQ(a1.value().motion_chain.size(), 3u);
    EXPECT_EQ(a1.value().executing_line, 0);
    seq.onActionCompleted();

    // WaitTime breaks the run.
    auto a2 = seq.advance(WorldSample{});
    ASSERT_TRUE(a2.isSuccess());
    EXPECT_EQ(a2.value().kind, StepActionKind::StartWaitTime);
    EXPECT_DOUBLE_EQ(a2.value().wait_duration.value(), 1.5);
    EXPECT_EQ(a2.value().executing_line, 3);
    seq.onActionCompleted();

    // The trailing single motion step.
    auto a3 = seq.advance(WorldSample{});
    ASSERT_TRUE(a3.isSuccess());
    EXPECT_EQ(a3.value().kind, StepActionKind::PlanMotionChain);
    EXPECT_EQ(a3.value().motion_chain.size(), 1u);
    EXPECT_EQ(a3.value().executing_line, 4);
    seq.onActionCompleted();

    EXPECT_EQ(seq.advance(WorldSample{}).value().kind, StepActionKind::Finished);
}

TEST(ProgramSequencer, CommentAndLabelAreInstant) {
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({
        makeComment(0), makeLabel(1), makeMotion(2, StepType::MoveL),
    })).isSuccess());

    auto a = seq.advance(WorldSample{});
    ASSERT_TRUE(a.isSuccess());
    EXPECT_EQ(a.value().kind, StepActionKind::PlanMotionChain);
    EXPECT_EQ(a.value().executing_line, 2);
    EXPECT_EQ(a.value().motion_chain.front().id, 2u);
}

// ============================================================================
// Loops (unconditional jump)
// ============================================================================

TEST(ProgramSequencer, UnconditionalJumpLoopsBack) {
    // Label(10) -> MoveL -> GOTO 10 : an endless author loop. Each iteration must yield the motion
    // action again (the loop exit is normally STOP, or a counter/IF in richer phases).
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({
        makeLabel(10), makeMotion(1, StepType::MoveL), makeGoto(2, 10),
    })).isSuccess());

    for (int iteration = 0; iteration < 5; ++iteration) {
        auto a = seq.advance(WorldSample{});
        ASSERT_TRUE(a.isSuccess()) << "iteration " << iteration;
        EXPECT_EQ(a.value().kind, StepActionKind::PlanMotionChain) << "iteration " << iteration;
        EXPECT_EQ(a.value().executing_line, 1) << "iteration " << iteration;
        seq.onActionCompleted();
    }
}

// ============================================================================
// Branches (conditional jump on a digital input)
// ============================================================================

// Program: MoveL(0) ; IF DI3==HIGH GOTO L(5) ; MoveJ(2) ; L: Label(5) ; MoveL(4)
// DI3 HIGH  -> the IF is taken, MoveJ(2) is skipped, execution continues at MoveL(4).
// DI3 LOW   -> the IF falls through, MoveJ(2) executes.
static ProgramDataStruct branchProgram() {
    return makeProgram({
        makeMotion(0, StepType::MoveL),
        makeIf(1, /*io_port*/ 3, /*trigger_on_state*/ true, /*target*/ 5),
        makeMotion(2, StepType::MoveJ),
        makeLabel(5),
        makeMotion(4, StepType::MoveL),
    });
}

TEST(ProgramSequencer, ConditionalJumpTakenSkipsBranch) {
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(branchProgram()).isSuccess());

    // First motion.
    ASSERT_EQ(seq.advance(WorldSample{}).value().kind, StepActionKind::PlanMotionChain);
    seq.onActionCompleted();

    // DI3 HIGH -> branch taken -> next motion is MoveL(4), MoveJ(2) skipped.
    WorldSample world;
    world.digital_inputs = diBit(3);
    auto a = seq.advance(world);
    ASSERT_TRUE(a.isSuccess());
    EXPECT_EQ(a.value().kind, StepActionKind::PlanMotionChain);
    EXPECT_EQ(a.value().executing_line, 4);
    EXPECT_EQ(a.value().motion_chain.front().id, 4u);
}

TEST(ProgramSequencer, ConditionalJumpNotTakenFallsThrough) {
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(branchProgram()).isSuccess());

    ASSERT_EQ(seq.advance(WorldSample{}).value().kind, StepActionKind::PlanMotionChain);
    seq.onActionCompleted();

    // DI3 LOW -> fall through -> MoveJ(2) executes.
    auto a = seq.advance(WorldSample{}); // digital_inputs == 0
    ASSERT_TRUE(a.isSuccess());
    EXPECT_EQ(a.value().kind, StepActionKind::PlanMotionChain);
    EXPECT_EQ(a.value().executing_line, 2);
    EXPECT_EQ(a.value().motion_chain.front().id, 2u);
}

TEST(ProgramSequencer, ConditionalJumpTriggersOnLowState) {
    // trigger_on_state == false means "jump when the input is LOW".
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({
        makeIf(0, /*io_port*/ 1, /*trigger_on_state*/ false, /*target*/ 7),
        makeMotion(1, StepType::MoveJ), // skipped when DI1 is LOW
        makeLabel(7),
        makeMotion(3, StepType::MoveL),
    })).isSuccess());

    auto a = seq.advance(WorldSample{}); // DI1 LOW -> condition satisfied -> jump
    ASSERT_TRUE(a.isSuccess());
    EXPECT_EQ(a.value().executing_line, 3);
    EXPECT_EQ(a.value().motion_chain.front().id, 3u);
}

TEST(ProgramSequencer, ConditionalJumpOnInvalidPortDoesNotJump) {
    // A condition on an out-of-range port cannot be verified: fail-safe means DO NOT jump.
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({
        makeIf(0, /*io_port*/ 0, /*trigger_on_state*/ true, /*target*/ 9), // port 0 is invalid
        makeMotion(1, StepType::MoveJ),
        makeLabel(9),
        makeMotion(3, StepType::MoveL),
    })).isSuccess());

    WorldSample world;
    world.digital_inputs = 0xFFFFFFFFu; // even with all inputs HIGH, an invalid port must not jump
    auto a = seq.advance(world);
    ASSERT_TRUE(a.isSuccess());
    EXPECT_EQ(a.value().executing_line, 1); // fell through to MoveJ(1)
    EXPECT_EQ(a.value().motion_chain.front().id, 1u);
}

// ============================================================================
// WaitDI — blocking wait on a digital input
// ============================================================================

TEST(ProgramSequencer, WaitDIBlocksUntilInputThenProceeds) {
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({
        makeWaitDI(0, /*io_port*/ 4, /*trigger_on_state*/ true, /*timeout_s*/ 5.0),
        makeMotion(1, StepType::MoveL),
    })).isSuccess());

    // Input LOW -> keep blocking, carrying the condition and timeout for the controller.
    WorldSample low; // digital_inputs == 0
    auto w1 = seq.advance(low);
    ASSERT_TRUE(w1.isSuccess());
    EXPECT_EQ(w1.value().kind, StepActionKind::WaitForInput);
    EXPECT_EQ(w1.value().wait_condition.io_port, 4);
    EXPECT_TRUE(w1.value().wait_condition.trigger_on_state);
    EXPECT_DOUBLE_EQ(w1.value().wait_timeout.value(), 5.0);
    EXPECT_EQ(w1.value().executing_line, 0);

    // Still LOW -> still blocking.
    EXPECT_EQ(seq.advance(low).value().kind, StepActionKind::WaitForInput);

    // Input HIGH -> the same call resolves the wait and proceeds to the motion step.
    WorldSample high;
    high.digital_inputs = diBit(4);
    auto a = seq.advance(high);
    ASSERT_TRUE(a.isSuccess());
    EXPECT_EQ(a.value().kind, StepActionKind::PlanMotionChain);
    EXPECT_EQ(a.value().executing_line, 1);
}

TEST(ProgramSequencer, WaitDITriggersOnLowState) {
    // trigger_on_state == false means "wait until the input goes LOW".
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({
        makeWaitDI(0, /*io_port*/ 2, /*trigger_on_state*/ false, /*timeout_s*/ 0.0),
        makeMotion(1, StepType::MoveJ),
    })).isSuccess());

    WorldSample high;
    high.digital_inputs = diBit(2); // port 2 HIGH -> condition (wait for LOW) not yet met
    EXPECT_EQ(seq.advance(high).value().kind, StepActionKind::WaitForInput);

    WorldSample low; // port 2 LOW -> condition met
    auto a = seq.advance(low);
    ASSERT_TRUE(a.isSuccess());
    EXPECT_EQ(a.value().kind, StepActionKind::PlanMotionChain);
    EXPECT_EQ(a.value().executing_line, 1);
}

TEST(ProgramSequencer, WaitDIReArmsAndReBlocksEachLoopIteration) {
    // Label(10) -> WaitDI(port6) -> MoveL -> GOTO 10. Each loop must wait for the input afresh.
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({
        makeLabel(10),
        makeWaitDI(1, /*io_port*/ 6, /*trigger_on_state*/ true, /*timeout_s*/ 0.0),
        makeMotion(2, StepType::MoveL),
        makeGoto(3, 10),
    })).isSuccess());

    WorldSample low;              // input released
    WorldSample high;
    high.digital_inputs = diBit(6); // input asserted

    for (int iteration = 0; iteration < 3; ++iteration) {
        // At the top of each iteration the WaitDI blocks while the input is LOW.
        EXPECT_EQ(seq.advance(low).value().kind, StepActionKind::WaitForInput) << "iteration " << iteration;
        // Asserting the input releases the wait and yields the motion step.
        auto a = seq.advance(high);
        ASSERT_TRUE(a.isSuccess()) << "iteration " << iteration;
        EXPECT_EQ(a.value().kind, StepActionKind::PlanMotionChain) << "iteration " << iteration;
        EXPECT_EQ(a.value().executing_line, 2) << "iteration " << iteration;
        seq.onActionCompleted();
    }
}

// ============================================================================
// Runaway-loop watchdog
// ============================================================================

TEST(ProgramSequencer, RunawayLoopFaultsAndLatches) {
    // Label(1) -> GOTO 1 : an instant loop with no motion or wait. It must fault, not hang.
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({makeLabel(1), makeGoto(1, 1)})).isSuccess());

    auto a = seq.advance(WorldSample{});
    ASSERT_TRUE(a.isError());
    EXPECT_EQ(a.error(), SequencerError::RunawayLoop);

    // The fault latches until reset()/load().
    auto a2 = seq.advance(WorldSample{});
    ASSERT_TRUE(a2.isError());
    EXPECT_EQ(a2.error(), SequencerError::RunawayLoop);
}

// ============================================================================
// reset()
// ============================================================================

TEST(ProgramSequencer, ResetRerunsFromStart) {
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({makeMotion(0, StepType::MoveL), makeWait(1, 0.5)})).isSuccess());

    // Run to completion.
    ASSERT_EQ(seq.advance(WorldSample{}).value().kind, StepActionKind::PlanMotionChain);
    seq.onActionCompleted();
    ASSERT_EQ(seq.advance(WorldSample{}).value().kind, StepActionKind::StartWaitTime);
    seq.onActionCompleted();
    ASSERT_EQ(seq.advance(WorldSample{}).value().kind, StepActionKind::Finished);

    // Reset re-arms from step 0.
    seq.reset();
    EXPECT_EQ(seq.currentLine(), -1);
    auto a = seq.advance(WorldSample{});
    ASSERT_TRUE(a.isSuccess());
    EXPECT_EQ(a.value().kind, StepActionKind::PlanMotionChain);
    EXPECT_EQ(a.value().executing_line, 0);
}

// ============================================================================
// P4: integer registers (SetVar/IncVar/DecVar), register-compare IF, BREAK
// ============================================================================

TEST(ProgramSequencer, RegisterCounterLoopRunsExactlyNIterations) {
    // The canonical counter loop (boss decision: authored GOTO+counter, no RUN-flag mechanism):
    //   0: SetVar R0=2 | 1: LABEL 1 | 2: MoveJ | 3: DecVar R0 | 4: IF R0>0 GOTO 1
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({
        makeSetVar(0, /*reg*/ 0, /*value*/ 2),
        makeLabel(1),
        makeMotion(2, StepType::MoveJ),
        makeDecVar(3, /*reg*/ 0),
        makeIfReg(4, /*reg*/ 0, CompareOp::GreaterThan, /*operand*/ 0, /*label*/ 1)
    })).isSuccess());

    // Iteration 1: registers are consumed as instant steps; the motion is the first action.
    auto a1 = seq.advance(WorldSample{});
    ASSERT_TRUE(a1.isSuccess());
    ASSERT_EQ(a1.value().kind, StepActionKind::PlanMotionChain);
    EXPECT_EQ(a1.value().executing_line, 2);
    seq.onActionCompleted();

    // Iteration 2: DecVar made R0=1, IF 1>0 jumped back to the label -> the motion again.
    auto a2 = seq.advance(WorldSample{});
    ASSERT_TRUE(a2.isSuccess());
    ASSERT_EQ(a2.value().kind, StepActionKind::PlanMotionChain);
    EXPECT_EQ(a2.value().executing_line, 2);
    seq.onActionCompleted();

    // DecVar made R0=0, IF 0>0 fell through -> the program is complete after exactly 2 iterations.
    auto a3 = seq.advance(WorldSample{});
    ASSERT_TRUE(a3.isSuccess());
    EXPECT_EQ(a3.value().kind, StepActionKind::Finished);
    EXPECT_EQ(seq.registers()[0], 0);
}

TEST(ProgramSequencer, RegisterCompareOpsEvaluateCorrectly) {
    // SetVar R3=5, then one IF per operator; a taken branch jumps over a BREAK to the end label.
    // Equal (5==5) is exercised as taken; NotEqual (5!=5) as not taken (falls through to motion).
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({
        makeSetVar(0, 3, 5),
        makeIfReg(1, 3, CompareOp::NotEqual, 5, /*label*/ 9), // 5!=5 -> NOT taken
        makeMotion(2, StepType::MoveJ),                        // reached because the IF fell through
        makeIfReg(3, 3, CompareOp::Equal, 5, /*label*/ 9),     // 5==5 -> taken, skips the BREAK
        makeBreak(4),
        makeLabel(9)
    })).isSuccess());

    auto a1 = seq.advance(WorldSample{});
    ASSERT_TRUE(a1.isSuccess());
    ASSERT_EQ(a1.value().kind, StepActionKind::PlanMotionChain); // NotEqual fell through
    seq.onActionCompleted();

    auto a2 = seq.advance(WorldSample{});
    ASSERT_TRUE(a2.isSuccess());
    EXPECT_EQ(a2.value().kind, StepActionKind::Finished); // Equal jumped over the BREAK
}

TEST(ProgramSequencer, BreakStopsProgramAtItsLine) {
    // 0: MoveJ | 1: Break | 2: MoveJ — the second motion must never be reached.
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({
        makeMotion(0, StepType::MoveJ), makeBreak(1), makeMotion(2, StepType::MoveJ)
    })).isSuccess());

    auto a1 = seq.advance(WorldSample{});
    ASSERT_TRUE(a1.isSuccess());
    ASSERT_EQ(a1.value().kind, StepActionKind::PlanMotionChain);
    ASSERT_EQ(a1.value().motion_chain.size(), 1u); // Break interrupts the contiguous motion run
    seq.onActionCompleted();

    auto a2 = seq.advance(WorldSample{});
    ASSERT_TRUE(a2.isSuccess());
    EXPECT_EQ(a2.value().kind, StepActionKind::Break);
    EXPECT_EQ(a2.value().executing_line, 1);

    // Terminal for this run: a further advance reports Finished, never the trailing motion.
    auto a3 = seq.advance(WorldSample{});
    ASSERT_TRUE(a3.isSuccess());
    EXPECT_EQ(a3.value().kind, StepActionKind::Finished);
}

TEST(ProgramSequencer, LoadRejectsRegisterIndexOutOfRange) {
    ProgramSequencer seq;
    const auto r1 = seq.load(makeProgram({makeSetVar(0, ProgramSequencer::kRegisterCount, 1)}));
    ASSERT_TRUE(r1.isError());
    EXPECT_EQ(r1.error(), SequencerError::InvalidRegisterIndex);

    const auto r2 = seq.load(makeProgram({
        makeLabel(1),
        makeIfReg(1, ProgramSequencer::kRegisterCount, CompareOp::Equal, 0, 1)
    }));
    ASSERT_TRUE(r2.isError());
    EXPECT_EQ(r2.error(), SequencerError::InvalidRegisterIndex);
}

TEST(ProgramSequencer, LoadRejectsRegisterSourcedWaitDi) {
    // A register only changes via program steps, so waiting on one either passes instantly or
    // hangs forever — authoring nonsense, refused fail-closed at load.
    ProgramStepStruct wait = makeWaitDI(0, /*port*/ 1, /*state*/ true, /*timeout*/ 0.0);
    wait.condition.source = ConditionSource::Register;
    ProgramSequencer seq;
    const auto r = seq.load(makeProgram({wait}));
    ASSERT_TRUE(r.isError());
    EXPECT_EQ(r.error(), SequencerError::UnsupportedStep);
}

TEST(ProgramSequencer, RegisterOnlyLoopHitsRunawayWatchdog) {
    // Label(1) -> IncVar -> GOTO 1: register steps are instant, so a loop with no motion/wait must
    // fault on the watchdog instead of spinning the tick forever.
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({
        makeLabel(1), makeIncVar(1, 0), makeGoto(2, 1)
    })).isSuccess());

    auto a = seq.advance(WorldSample{});
    ASSERT_TRUE(a.isError());
    EXPECT_EQ(a.error(), SequencerError::RunawayLoop);
}

TEST(ProgramSequencer, ResetClearsRegisters) {
    // A fresh RUN must start with deterministic counters: R0 set by the previous run is cleared.
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({makeSetVar(0, 0, 7), makeMotion(1, StepType::MoveJ)})).isSuccess());

    ASSERT_EQ(seq.advance(WorldSample{}).value().kind, StepActionKind::PlanMotionChain);
    EXPECT_EQ(seq.registers()[0], 7);

    seq.reset();
    EXPECT_EQ(seq.registers()[0], 0);
}

TEST(ProgramSequencer, LastBranchAnnotationTracksIfEvaluations) {
    // P5: the sequencer reports WHICH IF was evaluated last and whether it jumped; reset clears it.
    // 0: SetVar R0=1 | 1: LABEL 1 | 2: MoveJ | 3: DecVar R0 | 4: IF R0>0 GOTO 1
    ProgramSequencer seq;
    ASSERT_TRUE(seq.load(makeProgram({
        makeSetVar(0, 0, 1),
        makeLabel(1),
        makeMotion(2, StepType::MoveJ),
        makeDecVar(3, 0),
        makeIfReg(4, 0, CompareOp::GreaterThan, 0, 1)
    })).isSuccess());

    EXPECT_EQ(seq.lastBranchLine(), -1); // nothing evaluated yet

    ASSERT_EQ(seq.advance(WorldSample{}).value().kind, StepActionKind::PlanMotionChain);
    seq.onActionCompleted();
    // DecVar made R0=0; IF 0>0 fell through -> program finished, branch recorded as NOT taken.
    ASSERT_EQ(seq.advance(WorldSample{}).value().kind, StepActionKind::Finished);
    EXPECT_EQ(seq.lastBranchLine(), 4);
    EXPECT_FALSE(seq.lastBranchTaken());

    seq.reset();
    EXPECT_EQ(seq.lastBranchLine(), -1);
    EXPECT_FALSE(seq.lastBranchTaken());
}

// --- END OF FILE: HexaMotion/modules/program_sequencer/tests/gtest_program_sequencer_main.cpp ---
