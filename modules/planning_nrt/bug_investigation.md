# Bug Investigation Analysis

## Symptoms
1. **Test Failure**: `TrajectoryPlannerIntegrationTest.AddSingleTargetAndExecute` and `StreamingTargetsAreQueued` fail.
2. **Key Observation**: 
   - `Target Pos` increases as expected.
   - `Actual Pos` remains stuck at `0`.
   - `SimDriver` receives `0.00 deg` in `write()` consistently.
   - `HardwareManager` logs confirm that it is limiting the axis.

## Detailed Log Analysis
From the latest logs:
```
[HardwareManager] dt_s: 0.049648
[HardwareManager] Limiting axis 0. Delta: 0.001440, MaxDelta: 0.000000, LimitVel: 0.000000
```

## The Root Cause
In `HardwareManager.h`:
```cpp
class HardwareManager {
    // ...
    const InterfaceConfig& config_;
    const RobotLimits& limits_;
    // ...
};
```
It stores `limits_` as a **const reference**.

In `tests/gtest_trajectory_planner_main.cpp`:
```cpp
    void SetUp() override {
        // ...
        RobotLimits limits; // <--- Local variable on stack
        limits.joint_position_limits_deg.fill({-180.0_deg, 180.0_deg});
        limits.joint_velocity_limits_deg_s.fill(100.0_deg_s);
        auto hw_manager = std::make_shared<HardwareManager>(config, limits);
        // ...
        motion_manager = std::make_shared<MotionManager>(hw_manager, 40, limits, 20.0_deg);
        // ...
    } // <--- limits is destroyed here
```
When `SetUp` returns, `limits` is destroyed. `hw_manager` (which is stored in the fixture member `motion_manager`) holds a dangling reference to `limits`.

When `write()` is called later in the test execution, `applyCommandGovernor` accesses `limits_.joint_velocity_limits_deg_s[i]`. Since the memory is invalid (or overwritten), it reads garbage. In this case, it seemingly reads `0` (which is common for stack memory that hasn't been reused yet or was zeroed).

Because `LimitVel` is read as 0, the governor clamps all movement to 0.

## Fix
The test needs to keep `limits` alive for the duration of the test.
We should make `limits` a member variable of the test fixture class `TrajectoryPlannerIntegrationTest`.

Also `InterfaceConfig config;` is also a local variable in `SetUp` and `HardwareManager` stores `const InterfaceConfig& config_;`. This is also a dangling reference!

**Action Plan:**
1. Modify `tests/gtest_trajectory_planner_main.cpp`.
2. Move `InterfaceConfig config` and `RobotLimits limits` to be protected members of `TrajectoryPlannerIntegrationTest`.
3. Re-run tests.
