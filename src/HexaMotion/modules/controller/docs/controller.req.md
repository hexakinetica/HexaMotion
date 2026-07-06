# Requirements for Module: `controller`

## 1. Functional Requirements

- **[!REQ] REQ-CTRL-01: Central Orchestrator**  
  The module must coordinate the planner, motion manager, HAL, and state synchronization into one control loop.

- **[!REQ] REQ-CTRL-02: Mode Management**  
  The controller must support Idle, Jog, Program, Homing, and Error modes and provide explicit transitions.

- **[!REQ] REQ-CTRL-03: Program Execution**  
  The controller must execute programs provided by `RobotState` and monitor their completion.

- **[!REQ] REQ-CTRL-04: Homing Orchestration**  
  The controller must run the homing sequence using HAL primitives and update `RobotState` with progress.

- **[!REQ] REQ-CTRL-05: State Sync**  
  The controller must keep `RobotState` in sync with planner feedback and HAL diagnostics.

- **[!REQ] REQ-CTRL-06: Responsive Jog**  
  Manual jog (JOG panel, `jogHalDirect == false`) must stay responsive and not accumulate latency when commands are issued repeatedly. `handleJogRequest` routes through `TrajectoryPlanner::retargetJog` (supersede), not `addTargetWaypoint` (append), so each jog replaces the previous one instead of queuing behind the `MotionManager`'s per-segment completion gate (see REQ-PLAN-08). The HAL-overlay direct path (`jogHalDirect == true` → `handleHalDirectJog`) is unchanged.

- **[!REQ] REQ-CTRL-07: Cartesian orientation jog is a rotation composition**  
  Cartesian jog (`WORLD`/`TOOL`) must apply an orientation increment (`Rx`/`Ry`/`Rz`) as a rotation composition, never as scalar addition onto an Euler (RPY) component. Euler addition is equivalent to an axis rotation only at zero orientation; at non-trivial wrist poses it rotates about the wrong axis and couples the angles (the historical erratic-orientation-jog defect). TOOL-frame orientation jog uses an intrinsic composition (`R' = R_tcp · R_delta`, `FrameTransformer::composePoses`); WORLD/BASE-frame orientation jog uses an extrinsic rotation about the fixed frame axis (`R' = R_axis(δ) · R_pose`, `FrameTransformer::rotateAboutWorldAxis`). Translation increments (`X`/`Y`/`Z`) remain a direct position-vector add.

- **[!REQ] REQ-CTRL-08: Cartesian jog is maintained-target, TCP-pivoted, and continuity-guarded (KUKA-style)**  
  The Cartesian jog must accumulate increments on a **maintained TCP target** and **IK seed** held by the controller (`jog_tcp_world_` / `jog_seed_joints_`), not re-derive the start pose from the lagging actual joints on every click. The maintained target re-syncs from the live robot only on a fresh jog (re-arm via `handleJogEnableCommand`) or when it has drifted beyond a threshold (external move). The increment rotates **about the TCP** (the active tool is applied, then removed for IK), so the TCP position stays fixed during an orientation jog (verified in monitor and 3D view). Each click performs **one IK** from the maintained seed with a **continuity guard**: an IK solution whose largest joint step exceeds a fixed limit (configuration flip / singularity) is rejected and the maintained target is not advanced — a single click must never produce a random large move. The committed step executes as a **JOINT** move to the IK solution (no per-interpolation Cartesian IK, which jumped near singularities).

---

## 2. Non-Functional Requirements (NFRs)

- **[!NFR] NFR-CTRL-01: Deterministic Control Loop**  
  The main update loop must run at a predictable rate and avoid allocations.

- **[!NFR] NFR-CTRL-02: Explicit Errors**  
  All errors are surfaced as `Result<T, ErrorCode>`; no exceptions.

---

## 3. Interfaces

- `RobotController::init(config)`
- `RobotController::update()`
- `RobotController::setMode(mode)`
- `RobotController::getDiagnostics()`

---

## 4. Architecture Summary

- Owns the high-level control loop that synchronizes planner, motion manager, and HAL.
- Consumes `RobotState` command requests and pushes feedback/diagnostics back.
- Executes program steps and homing sequences as state machines.

## 5. Test Recommendations

**Unit Tests**
- Mode transitions enforce correct guard conditions.
- Homing sequence fails on missing DI/limit signals.

**Integration Tests**
- Program execution path: enqueue trajectory → motion manager → feedback update.
- Emergency stop propagates to state and clears program queue.