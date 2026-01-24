# Requirements for Module: `kinematics_nrt`

## 1. Functional Requirements

### 1.1. Core Responsibilities
- [!REQ] REQ-KIN-01: **Kinematic Calculations**
  - **Description**: The module must provide robust and accurate Forward Kinematics (FK) and Inverse Kinematics (IK) calculations.
  - **Acceptance Criteria**: `KinematicSolver` interface provides `solveFK()` and `solveIK()` methods.

- [!REQ] REQ-KIN-02: **Frame Transformations**
  - **Description**: The module must provide utility functions to transform Cartesian poses between different coordinate frames (World, Base, Flange, TCP).
  - **Acceptance Criteria**: `FrameTransformer` class provides static methods `calculateTcpInWorld`, `calculateFlangeInWorld`, and `applyBaseTransform`.

- [!REQ] REQ-KIN-03: **Separation of Geometry and Limits**
  - **Description**: The geometric definition of the robot (kinematic chain) must be decoupled from its physical limits (position, velocity).
  - **Acceptance Criteria**: `KinematicModel` class is responsible only for geometry. `RobotLimits` struct (`shared/data_types`) defines physical constraints. The `KinematicSolver` is constructed using both.

### 1.2. Configuration and Flexibility
- [!REQ] REQ-KIN-04: **Model Loading from URDF**
  - **Description**: The `KinematicModel` must be configurable at runtime by loading a robot description from a URDF file string. Hardcoded models are permissible only for testing and bootstrapping.
  - **Acceptance Criteria**: `KinematicModel` provides a static factory method `createFromURDF()`.

### 1.3. Safety and Error Handling
- [!REQ] REQ-KIN-05: **Safe IK Interface**
  - **Description**: The Inverse Kinematics solver must never return an invalid or incorrect solution upon failure. Its interface must force the caller to explicitly handle error conditions.
  - **Acceptance Criteria**: `solveIK()` returns a `Result<AxisSet, IKError>`. On failure, it returns an error enum (`Unreachable`, `Singularity`, etc.) instead of a joint set.

- [!REQ] REQ-KIN-06: **Dependency Encapsulation**
  - **Description**: External library dependencies (e.g., KDL) must be hidden from the module's public interface to prevent dependency leakage.
  - **Acceptance Criteria**: Public header files (`.h` in `src/`) do not include any headers from the KDL library. This is achieved using the PIMPL idiom.

### 1.4. Testability
- [!REQ] REQ-KIN-07: **Mockable Interface**
  - **Description**: The module must provide a mock implementation of the `KinematicSolver` interface for testing dependent modules (`planning_nrt`, `controller`) in isolation.
  - **Acceptance Criteria**: A `MockKinematicSolver.h` is provided in the `tests/` directory.

## 2. Non-functional Requirements (NFR)

- [!REQ] REQ-KIN-NFR-01: **No Exceptions in Public API**
  - **Description**: The public API of the module must not throw exceptions for predictable errors (e.g., IK failure).
  - **Acceptance Criteria**: All error-prone functions return `Result<T, E>` or a similar non-exception-based error reporting mechanism.

- [!REQ] REQ-KIN-NFR-02: **Comprehensive Unit Testing**
  - **Description**: The module's functionality, especially the FK/IK solvers, must be covered by unit tests.
  - **Acceptance Criteria**: Tests must include FK->IK->FK round-trip validation and checks for handling unreachable targets.

## 3. Architecture & Diagrams

```mermaid
classDiagram
    class KinematicSolver {
        <<Interface>>
        +solveFK(joints) bool
        +solveIK(pose, seed) Result~AxisSet, IKError~
    }
    
    class KdlKinematicSolver {
        -unique_ptr~Impl~ pimpl_
    }

    class KinematicModel {
        +createFromURDF(str) unique_ptr~KinematicModel~
        #unique_ptr~Impl~ pimpl_
    }

    class RobotLimits {
        +joint_position_limits_deg
        +joint_velocity_limits_deg_s
    }

    class FrameTransformer {
        <<Utility>>
        +calculateTcpInWorld(flange, tool) CartPose
        +calculateFlangeInWorld(tcp, tool) CartPose
    }

    KdlKinematicSolver --|> KinematicSolver
    KdlKinematicSolver o-- KinematicModel : uses
    KdlKinematicSolver o-- RobotLimits : uses
 ```