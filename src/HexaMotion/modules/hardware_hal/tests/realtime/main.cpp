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
#include <future>
#include <iostream>

using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;

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

class RealtimeTest : public ::testing::Test {
protected:
    void SetUp() override {
        std::cout << "[INFO] Launching Python Emulator..." << std::endl;
        auto script_path = locateRobotUtility();
        ASSERT_FALSE(script_path.empty()) << "robot_utility.py not found from cwd: " << std::filesystem::current_path();
        std::system(buildEmulatorCommand(script_path).c_str());
        std::this_thread::sleep_for(1500ms);

        config.realtime_type = InterfaceConfig::RealtimeInterfaceType::Udp;
        config.udp_control_config.remote_ip = "127.0.0.1";
        config.udp_control_config.remote_port = 50001;
        config.udp_control_config.local_port = 50002;
        config.simulation_initial_joints.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
        config.debug_stream_config.enabled = false;
        // Valid position limits required: SimDriver seeds its HAL soft limits from RobotLimits
        // (REQ-SIMMC-09); a default (0,0) pair would clamp every governed command to zero.
        limits.joint_position_limits_deg.fill({-180.0_deg, 180.0_deg});
        limits.joint_velocity_limits_deg_s.fill(1000.0_deg_s);
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

    bool WaitForFeedback(HardwareManager& hm, std::function<bool(const HardwareFeedback&)> condition, std::chrono::milliseconds timeout = 2000ms) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < timeout) {
            auto res = hm.read();
            if (res.isSuccess() && condition(res.value())) {
                return true;
            }
            std::this_thread::sleep_for(10ms);
        }
        return false;
    }

    InterfaceConfig config;
    RobotLimits limits;
};

TEST_F(RealtimeTest, ModeSwitchFailAndSync) {
    std::cout << "[INFO] Testing Mode Switch Fail & Sync..." << std::endl;
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());

    AxisSet sim_cmd;
    sim_cmd.SetFromPositionArray({50.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    hm.write(sim_cmd);

    // Sim is at 50. Real is at 0 (init).
    // Tick the read path like the production RT loop does: SimDriver models a 1-cycle drive
    // latency, so without a read the setMode sync check would compare against the stale
    // pre-write sim pose and wave the switch through.
    (void)hm.read();
    (void)hm.read();
    std::this_thread::sleep_for(200ms);

    std::cout << "[INFO] Attempting switch to Realtime (should fail)..." << std::endl;
    auto switch_res = hm.setMode(HalMode::Realtime);
    // ASSERT (not EXPECT): calling .error() on a Success Result aborts the whole process,
    // so the test must stop here if the switch unexpectedly succeeded.
    ASSERT_TRUE(switch_res.isError()) << "Switch should fail due to large diff";
    EXPECT_EQ(switch_res.error(), ErrorCode::InvalidArgument);

    std::cout << "[INFO] Syncing Sim to Real..." << std::endl;
    hm.syncSimulationToReal();
    
    auto fb_res_sim = hm.read();
    ASSERT_TRUE(fb_res_sim.isSuccess());
    EXPECT_NEAR(fb_res_sim.value().joints.GetAt(0).value().get().position.value(), 0.0, 1.0) << "Sim should be synced to 0";

    std::cout << "[INFO] Attempting switch to Realtime (should success)..." << std::endl;
    EXPECT_TRUE(hm.setMode(HalMode::Realtime).isSuccess());
}

TEST_F(RealtimeTest, ZeroAxis) {
    std::cout << "[INFO] Testing ZeroAxis (Mastering) in Realtime..." << std::endl;
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());
    hm.syncSimulationToReal();
    ASSERT_TRUE(hm.setMode(HalMode::Realtime).isSuccess());
    
    // Move to 25 deg
    std::cout << "[INFO] Moving to 25.0 degrees..." << std::endl;
    AxisSet cmd;
    cmd.SetFromPositionArray({25.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    hm.write(cmd);
    
    ASSERT_TRUE(WaitForFeedback(hm, [](auto& fb){ 
        return std::abs(fb.joints.GetAt(0).value().get().position.value() - 25.0) < 1.0; 
    })) << "Timeout waiting to reach 25.0";

    // Zero it
    std::cout << "[INFO] Zeroing Axis 1..." << std::endl;
    ASSERT_TRUE(hm.zeroAxis(AxisId::A1).isSuccess());

    // Logical should become 0
    ASSERT_TRUE(WaitForFeedback(hm, [](auto& fb){ 
        return std::abs(fb.joints.GetAt(0).value().get().position.value()) < 1.0; 
    })) << "Timeout waiting for logical 0 after zeroing";
    
    auto fb = hm.read().value();
    std::cout << "[INFO] Final Logical Pos: " << fb.joints.GetAt(0).value().get().position.value() << std::endl;
}

int main(int argc, char **argv) {
    // Console sink so HAL/driver warnings (send failures, mode switches) are visible in
    // the test log — this suite drives real sockets and an external emulator process.
    RDT::Logger::Init({std::make_shared<RDT::ConsoleSink>()}, RDT::LogLevel::Info);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
