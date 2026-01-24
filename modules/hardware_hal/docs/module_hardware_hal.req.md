# Requirements for Module: hardware_hal

## 1. Functional Requirements

### 1.1. Core Responsibilities

#### [!REQ] REQ-HAL-01: Single Responsibility (Cyclic IO)
**Description:**  
The module operates strictly as a Real-Time IO layer. It provides a cyclic exchange of logical position commands (`AxisSet`) and feedback (`HardwareFeedback`). High-level logic (scripts, complex procedures) is strictly forbidden.

**Acceptance Criteria:**  
The public API of `HardwareManager` exposes only:
- `read()`
- `write()`
- `setMode()`
- `syncSimulationToReal()`
- `zeroAxis()`

---

#### [!REQ] REQ-HAL-02: Driver Abstraction
**Description:**  
The module must provide a single entry point `HardwareManager` that abstracts the underlying active driver (Simulation, EtherCAT, UDP).

**Acceptance Criteria:**  
The consuming code (`MotionManager`) must not know or care which driver is currently active.

---

### 1.2. Safety & Protection (The Governor)

#### [!REQ] REQ-HAL-03: Command Governor (Velocity Limiting)
**Description:**  
`HardwareManager` must internally enforce velocity limits. If a command implies a velocity exceeding the physical limits of the robot (defined in `RobotLimits`), the command sent to the hardware must be clamped to the maximum allowed velocity for the cycle.

**Acceptance Criteria:**  
Sending a large step command (e.g., instant jump `0 → 90 deg`) results in the hardware receiving a sequence of smaller position commands that ramp up at the maximum allowed velocity.

---

#### [!REQ] REQ-HAL-04: Stateless Governor Logic
**Description:**  
The Governor must not use a queue. It must validate the current target against the last successfully sent command to calculate the required velocity.

**Acceptance Criteria:**  
Old commands are discarded; the system always tries to reach the most recent target safely.

---

### 1.3. Mode Management

#### [!REQ] REQ-HAL-05: Simulation Mode Default
**Description:**  
Upon initialization, the HAL must default to Simulation mode using an internal mathematical model (`SimDriver`).

**Acceptance Criteria:**  
`getMode()` returns `Simulation` after `init()`.

---

#### [!REQ] REQ-HAL-06: Safe Mode Switching (Sim → Real)
**Description:**  
Switching from Simulation to Realtime is only permitted if the simulated logical pose matches the real logical pose within a configurable tolerance.

**Acceptance Criteria:**  
`setMode(Realtime)` returns `ErrorCode::NotInSync` (or equivalent) if the deviation is greater than the tolerance.

---

#### [!REQ] REQ-HAL-07: Sync Simulation to Real
**Description:**  
The module must provide a method (`syncSimulationToReal`) to instantaneously snap the internal simulation state to the current real hardware state.

**Acceptance Criteria:**  
After `syncSimulationToReal()`, the deviation between simulated and real logical positions is approximately zero.

---

### 1.4. Hardware Interaction & Homing Primitives

#### [!REQ] REQ-HAL-08: Raw Digital Inputs for Homing
**Description:**  
The HAL must expose a raw bitmask of Digital Inputs (DI) in the `HardwareFeedback` struct every cycle. This is a required primitive for higher-level Homing procedures (e.g., via optical sensors).

**Acceptance Criteria:**  
Changing a physical DI on the bus is reflected in `HardwareFeedback.digital_inputs` within one cycle.

---

#### [!REQ] REQ-HAL-09: Encoder Zeroing (Mastering Primitive)
**Description:**  
The module must provide a method (`zeroAxis`) to establish the logical zero position for an axis. This method does not move the robot. It recalculates an internal offset such that the current physical encoder reading corresponds to the new logical zero. This is the core primitive for all Mastering and Homing procedures.

**Acceptance Criteria:**  
Calling `zeroAxis(A1)` results in the reported logical position of `A1` becoming `0.0` in the next feedback cycle, while the physical position remains unchanged.

---

### 1.5. Safety Monitoring (STO)

#### [!REQ] REQ-HAL-10: STO State Monitoring
**Description:**  
The module must report the status of the hardware safety chain (E-Stop, hard limits, power) via `SafetyState` flags. It is **not** responsible for the physical stop action (which is hardware-wired).

**Acceptance Criteria:**  
`HardwareFeedback.safety.is_estop_active` becomes `true` when the physical E-Stop button is pressed.

---

#### [!REQ] REQ-HAL-11: Hard Limit Monitoring
**Description:**  
The module must detect and report if a hardware limit switch is triggered.

**Acceptance Criteria:**  
`HardwareFeedback.safety.is_hard_limit_hit` becomes `true` when a limit switch is active.

---

### 1.6. Error Handling

#### [!REQ] REQ-HAL-12: Explicit Result Types
**Description:**  
All methods capable of failure must return `Result<T, ErrorCode>`. Exceptions are strictly forbidden.

**Acceptance Criteria:**  
Network disconnects return `ErrorCode::NotConnected` or a similarly specific error.

---

#### [!REQ] REQ-HAL-13: Diagnostic Status
**Description:**  
`HardwareFeedback` must include a `HalStatus` enum to report driver health (e.g., `Ok`, `Warning_SyncLost`, `Error_DriveFault`).

**Acceptance Criteria:**  
A lost packet (if recovered) reports `Warning`; a broken connection reports `Error`.

---

## 2. Non-functional Requirements (NFR)

#### [!REQ] REQ-HAL-NFR-01: Wait-Free Hot Path
**Description:**  
The `read()` and `write()` methods used in the RT-loop must be wait-free or lock-free to ensure deterministic execution.

**Acceptance Criteria:**  
No mutex locks and no heap allocations (`new`, `malloc`) in the cyclic path.

---

#### [!REQ] REQ-HAL-NFR-02: Thread Safety
**Description:**  
The internal drivers must handle thread safety for their background polling threads without blocking the RT-thread calling `read()` / `write()`.

**Acceptance Criteria:**  
Use of `std::atomic` for data exchange between the driver’s background thread and the HAL API methods.

---

## 3. Architecture & Diagrams

### 3.1. Flat Architecture

The module follows a flat, facade-based architectural pattern.  
`HardwareManager` is the **only public entry point** to the Hardware Abstraction Layer.

All hardware-specific logic is encapsulated behind the internal `IDriver` interface.  
Concrete `IDriver` implementations (Simulation, UDP, EtherCAT) are responsible for:

- Communication with the physical or simulated hardware
- Maintaining the mapping between **physical** and **logical** coordinates
- Applying axis-specific offsets internally

The relationship between physical and logical coordinates is defined as:
LogicalPosition = PhysicalPosition + Offset


This design ensures:
- Strict separation between high-level motion logic and low-level hardware access
- Driver interchangeability without changes to consuming modules
- Deterministic and testable behavior of the HAL



