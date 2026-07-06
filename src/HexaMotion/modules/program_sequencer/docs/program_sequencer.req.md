# Module requirements: program_sequencer (P1 + WaitDI logic)

- **Status:** implemented standalone (19 unit tests green in isolation); NOT integrated into HexaMotion.
- **Scope:** controller-side execution of the authored flat program — the "counterpart" to the pendant
  trajectory-setter (`hexa::ProgramBuilder`). Cross-module design and rationale live in the upper-level
  document `docs/REQ_program_sequencer.md`; this file states only the module-own requirements.

## What this module is

`RDT::ProgramSequencer` is a pure, exception-free interpreter of `RDT::NetProtocol::ProgramDataStruct`.
It owns the program counter, the label table and a runaway-loop watchdog, and returns the next
`StepAction` for the controller to actuate. It has no Qt, no hardware and no wall clock, so its
loop/branch/wait decisions are fully unit-tested offline.

## Requirements

- **SEQ-REQ-0001 (fail-closed load).** `load()` rejects — and leaves the sequencer unarmed for — an
  empty program, a duplicate `Label` id, any `JumpToLabel`/`ConditionalJump` whose `jump_target_id`
  has no matching `Label`, and any step type not executable in this phase. Each returns a typed
  `SequencerError`. Rationale: the controller is the final arbiter and must never silently mis-execute
  (kills the default "GOTO → step 0 → infinite loop").
- **SEQ-REQ-0002 (executable set).** Executable: `Comment`, `MoveJ`, `MoveL`, `WaitTime`, `WaitDI`,
  `Label`, `JumpToLabel`, `ConditionalJump`. Not executable (fail-closed): `None`, `MoveC` (needs
  planner), `SetDO` (P3, needs a HAL digital-output write path).
- **SEQ-REQ-0003 (motion chaining).** A maximal contiguous run of `MoveJ`/`MoveL` is gathered into one
  `PlanMotionChain` action so the controller plans it as a single continuous chain; a non-motion step
  breaks the run.
- **SEQ-REQ-0004 (loops).** `JumpToLabel` sets the program counter to the resolved label index,
  enabling author-defined loops.
- **SEQ-REQ-0005 (branches).** `ConditionalJump` jumps when its `Condition` is satisfied by the
  `WorldSample` digital-input bitmask (DI ports 1-based; bit = `io_port - 1`), else falls through. A
  condition on an out-of-range port is fail-safe (never jumps).
- **SEQ-REQ-0006 (iterative + watchdog).** Dispatch is iterative (no recursion). At most
  `kMaxInstantStepsPerTick` instant steps are consumed per `advance()`; exceeding it faults with
  `RunawayLoop` (latched until `reset()`/`load()`), so a mis-authored jump-only loop cannot overflow
  the stack or hang a control tick.
- **SEQ-REQ-0007 (completion is external).** A blocking action (motion/wait) is completed by the
  controller via `onActionCompleted()`; the sequencer owns no timing. `reset()` re-arms from step 0.
- **SEQ-REQ-0008 (display line).** `currentLine()` / `StepAction::executing_line` report the executing
  step index for the UI highlight (display only; never a control input).
- **SEQ-REQ-0009 (WaitDI blocking wait).** A `WaitDI` step emits `WaitForInput` and blocks until its
  `Condition` is satisfied by the `WorldSample` digital inputs. Unlike motion/time actions, it is
  condition-based and self-completes inside `advance()` (not via `onActionCompleted()`). The timeout is
  the step's `wait_duration_s` (0 = wait indefinitely), carried on every `WaitForInput` for the
  controller to enforce; the sequencer owns no clock. Timeout policy is the proposed answer to
  `docs/REQ_program_sequencer.md` §11, pending boss confirmation.

## Interfaces to the rest of the system (referenced, not owned here)

- **In:** `NetProtocol::ProgramDataStruct` (from the pendant via the RDT program transaction) and a
  `WorldSample` the controller distils from the circulating `TrajectoryPoint` (see
  `docs/REQ_program_sequencer.md` §3.3 — the `digital_inputs` enrichment is a P2 controller change).
- **Out:** `StepAction` the controller actuates on `TrajectoryPlanner`/HAL; `onActionCompleted()` closes
  the loop from motion/wait completion.

## Build & test

```
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure     # or run program_sequencer_test_app directly
```

## Not in this module (staged, see upper-level roadmap)

The WaitDI *decision logic* lives here, but the production pieces that feed it do not: the RT-loop
`RobotFeedbackFrame.digital_inputs` enrichment and the `mapHmiProgramToRdt` condition/jump-target fill
(P2), HAL digital output + SetDO (P3), integer registers + counter branches (P4, protocol bump),
execution annotation on the feedback loop (P5), and parametric/indexed points for palletizing (P6).
Integration into `RobotController` replaces the current `executeNextStep` switch and is a separate,
reviewed step.
