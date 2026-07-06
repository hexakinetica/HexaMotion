#include "gtest/gtest.h"
#include "HardwareManager.h"
#include "RobotConfig.h"
#include "Logger.h"
#include "LoggingMacros.h"
#include "Units.h"

#include <thread>
#include <chrono>
#include <cstdlib> 
#include <future>
#include <iostream>
#include <filesystem>

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
}

class SafetyTest : public ::testing::Test {
protected:
    void SetUp() override {
        std::cout << "[INFO] Launching Python Emulator..." << std::endl;
        auto script_path = locateRobotUtility();
        ASSERT_FALSE(script_path.empty()) << "robot_utility.py not found from cwd: " << std::filesystem::current_path();
        std::cout << "[INFO] Using emulator script: " << script_path << std::endl;
        auto command = buildEmulatorCommand(script_path);
        std::system(command.c_str());
        std::this_thread::sleep_for(2000ms);

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

    bool WaitForEmulatorReady(HardwareManager& hm, std::chrono::milliseconds timeout = 5000ms) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < timeout) {
            auto res = hm.read();
            if (res.isSuccess()) {
                std::cout << "[DEBUG] Emulator feedback: hal=" << static_cast<int>(res.value().driver_status)
                          << " estop=" << res.value().safety.is_estop_active
                          << " power=" << res.value().safety.is_power_on << std::endl;
                return true;
            }
            std::this_thread::sleep_for(50ms);
        }
        return false;
    }

    InterfaceConfig config;
    RobotLimits limits;
};

TEST_F(SafetyTest, STOAndSafetyMonitoring) {
    std::cout << "[INFO] Testing STO Status..." << std::endl;
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());
    ASSERT_TRUE(WaitForEmulatorReady(hm)) << "Emulator not ready";
    hm.syncSimulationToReal();
    ASSERT_TRUE(hm.setMode(HalMode::Realtime).isSuccess());

    // Wait for connection to be established and status to become Ok
    ASSERT_TRUE(WaitForFeedback(hm, [](const HardwareFeedback& fb){
        return fb.driver_status == HalStatus::Ok;
    }, 5000ms)) << "Timeout waiting for HalStatus::Ok";

    auto fb = hm.read().value();
    EXPECT_EQ(fb.driver_status, HalStatus::Ok);
    
    // By default emulator has power on
    EXPECT_TRUE(fb.safety.is_power_on) << "Emulator should start with POWER=ON";
    EXPECT_FALSE(fb.safety.is_estop_active) << "Emulator should start with ESTOP=OFF";
}

TEST_F(SafetyTest, CommunicationDiagnosticError) {
    std::cout << "[INFO] Testing Communication Loss..." << std::endl;
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());
    ASSERT_TRUE(WaitForEmulatorReady(hm)) << "Emulator not ready";
    hm.syncSimulationToReal();
    ASSERT_TRUE(hm.setMode(HalMode::Realtime).isSuccess());

    // Ensure connected first
    ASSERT_TRUE(WaitForFeedback(hm, [](const HardwareFeedback& fb){
        return fb.driver_status == HalStatus::Ok;
    }, 5000ms));

    std::cout << "[INFO] Killing emulator to simulate link loss..." << std::endl;
    TearDown(); // Kill emulator

    // Wait for error state
    ASSERT_TRUE(WaitForFeedback(hm, [](auto& fb){
        return fb.driver_status != HalStatus::Ok;
    }, 5000ms)) << "Timeout waiting for error status";
    
    auto fb = hm.read().value();
    EXPECT_TRUE(fb.driver_status == HalStatus::Error_CommunicationLost || fb.driver_status == HalStatus::Warning_SyncLost) 
        << "Status should be Error or Warning, got: " << (int)fb.driver_status;
}

int main(int argc, char **argv) {
    RDT::Logger::Init({}, RDT::LogLevel::Info);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
