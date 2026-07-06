#include "gtest/gtest.h"
#include "HardwareManager.h"
#include "RobotConfig.h"
#include "Logger.h"
#include "LoggingMacros.h"
#include "Units.h"

#include <thread>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iostream>

using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;

// The Command Governor clamps only in Realtime mode (internal simulation is a trajectory
// sandbox and is intentionally passed through). These tests therefore run against the UDP
// emulator (robot_utility.py), which teleports to the commanded position and echoes it back,
// so the feedback position equals the governor's clamped command.

namespace {
std::filesystem::path locateRobotUtility() {
    // Walk up from the cwd. On Windows parent_path() of a root ("D:\\") returns the root
    // itself, so the loop must stop when the path no longer shrinks (a `!p.empty()` guard
    // alone spins forever).
    auto p = std::filesystem::current_path();
    while (true) {
        // Module-local build layout (cwd inside src/HexaMotion).
        auto candidate = p / "modules" / "hardware_hal" / "tests" / "robot_utility.py";
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
        // Repo-root layout (cwd inside build/bin of the main tree).
        candidate = p / "src" / "HexaMotion" / "modules" / "hardware_hal" / "tests" / "robot_utility.py";
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
        auto parent = p.parent_path();
        if (parent == p) {
            return {};
        }
        p = parent;
    }
}

std::string buildEmulatorCommand(const std::filesystem::path& script_path) {
#ifdef _WIN32
    return "start /B python \"" + script_path.string() + "\" --emulator --debug > nul 2>&1";
#else
    return "python3 \"" + script_path.string() + "\" --emulator --debug > /dev/null 2>&1 &";
#endif
}
} // namespace

class GovernorTest : public ::testing::Test {
protected:
    void SetUp() override {
        std::cout << "[INFO] Launching Python Emulator..." << std::endl;
        auto script_path = locateRobotUtility();
        ASSERT_FALSE(script_path.empty()) << "robot_utility.py not found from cwd: " << std::filesystem::current_path();
        std::system(buildEmulatorCommand(script_path).c_str());
        std::this_thread::sleep_for(2000ms);

        config.realtime_type = InterfaceConfig::RealtimeInterfaceType::Udp;
        config.udp_control_config.remote_ip = "127.0.0.1";
        config.udp_control_config.remote_port = 50001;
        config.udp_control_config.local_port = 50002;
        config.simulation_initial_joints.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
        config.debug_stream_config.enabled = false;

        // Position limits must be sane too: SimDriver seeds its HAL soft limits from these
        // RobotLimits (REQ-SIMMC-09), and a default-constructed (0,0) pair would make the
        // governor clamp every commanded position to zero. Production sanitizes this
        // (ControllerRuntime::sanitizeAxisLimitsInPlace); the bench must ship a valid config.
        limits.joint_position_limits_deg.fill({-180.0_deg, 180.0_deg});
        limits.joint_velocity_limits_deg_s.fill(1000.0_deg_s);
        limits.joint_velocity_limits_deg_s[0] = 10.0_deg_s; // Reduce axis 1 limit to 10 deg/s
    }

    void TearDown() override {
        std::cout << "[INFO] Killing Python Emulator..." << std::endl;
#ifdef _WIN32
        std::system("taskkill /F /IM python.exe /T > nul 2>&1");
#else
        std::system("pkill -f 'robot_utility.py --emulator --debug' > /dev/null 2>&1");
#endif
        std::this_thread::sleep_for(200ms);
    }

    bool WaitForFeedback(HardwareManager& hm, std::function<bool(const HardwareFeedback&)> condition, std::chrono::milliseconds timeout = 5000ms) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < timeout) {
            auto res = hm.read();
            if (res.isSuccess() && condition(res.value())) {
                return true;
            }
            std::this_thread::sleep_for(20ms);
        }
        return false;
    }

    // Bring the HAL to Realtime mode with all motors enabled and the per-axis velocity limits
    // set explicitly through the production config path. The governor holds position for
    // disabled motors, and it prefers the HAL-config velocity limit over the RobotLimits
    // fallback whenever velocity_limit_enabled is set — which it is by default in the sim
    // driver state (seeded from RobotLimits since REQ-SIMMC-09) that seeds hal state before
    // the mode switch. Setting the limit here makes the test's 10 deg/s clamp authoritative.
    void EnterRealtimeWithMotorsEnabled(HardwareManager& hm) {
        ASSERT_TRUE(hm.init().isSuccess());
        ASSERT_TRUE(WaitForFeedback(hm, [](const HardwareFeedback& fb) {
            return fb.driver_status == HalStatus::Ok;
        })) << "Emulator not ready";
        (void)hm.syncSimulationToReal();
        ASSERT_TRUE(hm.setMode(HalMode::Realtime).isSuccess());

        HalConfigCommand enable_cmd;
        enable_cmd.requestId = 1;
        for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            auto& axis = enable_cmd.axes[i];
            axis.update_mask = HalConfigUpdate_MotorEnabled | HalConfigUpdate_VelocityLimit;
            axis.motor_enabled = true;
            axis.velocity_limit_enabled = true;
            axis.velocity_limit = limits.joint_velocity_limits_deg_s[i];
        }
        ASSERT_TRUE(hm.setHalConfig(enable_cmd).isSuccess());
    }

    InterfaceConfig config;
    RobotLimits limits;
};

TEST_F(GovernorTest, CommandGovernorClampsVelocity) {
    std::cout << "[INFO] Testing Positive Velocity Clamping..." << std::endl;
    HardwareManager hm(config, limits);
    EnterRealtimeWithMotorsEnabled(hm);

    AxisSet start_cmd;
    start_cmd.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    ASSERT_TRUE(hm.write(start_cmd).isSuccess());

    // Cycle time 200ms
    std::this_thread::sleep_for(200ms);

    // Limit is 10 deg/s. In 200ms, max move is 10 * 0.2 = 2.0 deg.
    // Target is 10.0 deg. Result should be close to 2.0 deg, not 10.0.
    AxisSet fast_cmd;
    fast_cmd.SetFromPositionArray({10.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    ASSERT_TRUE(hm.write(fast_cmd).isSuccess());

    // Allow slight tolerance for timing jitter plus the UDP echo round-trip.
    double last_pos = 0.0;
    EXPECT_TRUE(WaitForFeedback(hm, [&last_pos](const HardwareFeedback& fb) {
        last_pos = fb.joints.GetAt(0).value().get().position.value();
        return std::abs(last_pos - 2.0) < 0.5;
    }, 2000ms)) << "Axis 1 was not clamped to ~2.0 deg. mode="
                << (hm.getMode() == HalMode::Realtime ? "REAL" : "SIM")
                << " motor_enabled=" << hm.getCurrentHalState().axes[0].motor_enabled
                << " last_pos=" << last_pos;
}

TEST_F(GovernorTest, CommandGovernorClampsNegativeVelocity) {
    std::cout << "[INFO] Testing Negative Velocity Clamping..." << std::endl;
    HardwareManager hm(config, limits);
    EnterRealtimeWithMotorsEnabled(hm);

    AxisSet start_cmd;
    start_cmd.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    ASSERT_TRUE(hm.write(start_cmd).isSuccess());
    std::this_thread::sleep_for(200ms);

    // Command -10 deg. Limit 10 deg/s. In 200ms, max move is -2.0 deg.
    AxisSet neg_cmd;
    neg_cmd.SetFromPositionArray({-10.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    ASSERT_TRUE(hm.write(neg_cmd).isSuccess());

    EXPECT_TRUE(WaitForFeedback(hm, [](const HardwareFeedback& fb) {
        const double pos = fb.joints.GetAt(0).value().get().position.value();
        return std::abs(pos - (-2.0)) < 0.5;
    }, 2000ms)) << "Axis 1 was not clamped to ~-2.0 deg";
}

TEST_F(GovernorTest, MultiAxisGovernor) {
    std::cout << "[INFO] Testing Multi-Axis Governor..." << std::endl;
    HardwareManager hm(config, limits);
    EnterRealtimeWithMotorsEnabled(hm);

    AxisSet start_cmd;
    start_cmd.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    ASSERT_TRUE(hm.write(start_cmd).isSuccess());

    std::this_thread::sleep_for(10ms);

    // All axes commanded to 90 deg in one short (~10ms) step: every axis must be clamped below
    // the target (axis 1 by its tight 10 deg/s limit, the rest by the 1000 deg/s default) while
    // still making forward progress. The short cycle keeps 1000 deg/s * dt well below 80 deg
    // even with Windows timer jitter.
    AxisSet multi_cmd;
    multi_cmd.SetFromPositionArray({90.0_deg, 90.0_deg, 90.0_deg, 90.0_deg, 90.0_deg, 90.0_deg});
    ASSERT_TRUE(hm.write(multi_cmd).isSuccess());

    EXPECT_TRUE(WaitForFeedback(hm, [](const HardwareFeedback& fb) {
        for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            const double pos = fb.joints.GetAt(i).value().get().position.value();
            if (pos <= 0.0 || pos >= 80.0) {
                return false;
            }
        }
        return true;
    }, 2000ms)) << "Expected every axis clamped into (0, 80) deg after one governed cycle";
}

int main(int argc, char **argv) {
    // Console sink so HAL/driver warnings (send failures, mode switches) are visible in
    // the test log — this suite drives real sockets and an external emulator process.
    RDT::Logger::Init({std::make_shared<RDT::ConsoleSink>()}, RDT::LogLevel::Info);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
