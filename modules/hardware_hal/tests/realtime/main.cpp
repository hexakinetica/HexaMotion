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

using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;

class RealtimeTest : public ::testing::Test {
protected:
    void SetUp() override {
        std::cout << "[INFO] Launching Python Emulator..." << std::endl;
#ifdef _WIN32
        const char* command = "start /B python ../tests/robot_utility.py --emulator --debug > nul 2>&1";
#else
        const char* command = "python3 ../tests/robot_utility.py --emulator --debug > /dev/null 2>&1 &";
#endif
        std::system(command);
        std::this_thread::sleep_for(1500ms);

        config.realtime_type = InterfaceConfig::RealtimeInterfaceType::Udp;
        config.udp_control_config.remote_ip = "127.0.0.1";
        config.udp_control_config.remote_port = 50001;
        config.udp_control_config.local_port = 50002;
        config.simulation_initial_joints.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
        config.debug_stream_config.enabled = false;
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
    std::this_thread::sleep_for(200ms); 

    std::cout << "[INFO] Attempting switch to Realtime (should fail)..." << std::endl;
    auto switch_res = hm.setMode(HalMode::Realtime);
    EXPECT_TRUE(switch_res.isError()) << "Switch should fail due to large diff";

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
    RDT::Logger::Init({}, RDT::LogLevel::Info);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
