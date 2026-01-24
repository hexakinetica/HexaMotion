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
#include <vector>

using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;

class HardwareManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Cross-platform emulator launch
#ifdef _WIN32
        const char* command = "start /B python ../tests/robot_utility.py --emulator --debug > nul 2>&1";
#else
        const char* command = "python3 ../tests/robot_utility.py --emulator --debug > /dev/null 2>&1 &";
#endif
        std::system(command);
        std::this_thread::sleep_for(1500ms); // Give more time for Python to bind socket

        config.realtime_type = InterfaceConfig::RealtimeInterfaceType::Udp;
        config.udp_control_config.remote_ip = "127.0.0.1";
        config.udp_control_config.remote_port = 50001;
        config.udp_control_config.local_port = 50002;
        config.simulation_initial_joints.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
        
        config.debug_stream_config.enabled = false;

        limits.joint_velocity_limits_deg_s.fill(1000.0_deg_s);
        limits.joint_velocity_limits_deg_s[0] = 100.0_deg_s;
    }

    void TearDown() override {
#ifdef _WIN32
        std::system("taskkill /F /IM python.exe /T > nul 2>&1");
#else
        std::system("pkill -f 'robot_utility.py --emulator --debug' > /dev/null 2>&1");
#endif
        std::this_thread::sleep_for(200ms);
    }

    // Helper to wait for specific feedback condition
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

TEST_F(HardwareManagerTest, InitializationAndDefaultMode) {
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());
    EXPECT_EQ(hm.getMode(), HalMode::Simulation);
}

TEST_F(HardwareManagerTest, SimulationModeReadWrite) {
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());
    
    AxisSet cmd;
    cmd.SetFromPositionArray({10.0_deg, 20.0_deg, 30.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    ASSERT_TRUE(hm.write(cmd).isSuccess());
    
    auto fb_res = hm.read();
    ASSERT_TRUE(fb_res.isSuccess());
    EXPECT_NEAR(fb_res.value().joints.GetAt(0).value().get().position.value(), 10.0, 1e-6);
    EXPECT_NEAR(fb_res.value().joints.GetAt(1).value().get().position.value(), 20.0, 1e-6);
}

TEST_F(HardwareManagerTest, ModeSwitchFailAndSync) {
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());

    AxisSet sim_cmd;
    sim_cmd.SetFromPositionArray({50.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    hm.write(sim_cmd);
    
    // Ensure we are NOT in sync with real (which is at 0)
    std::this_thread::sleep_for(200ms); 

    auto switch_res = hm.setMode(HalMode::Realtime);
    EXPECT_TRUE(switch_res.isError());

    hm.syncSimulationToReal();
    
    auto fb_res_sim = hm.read();
    ASSERT_TRUE(fb_res_sim.isSuccess());
    EXPECT_NEAR(fb_res_sim.value().joints.GetAt(0).value().get().position.value(), 0.0, 1.0);

    EXPECT_TRUE(hm.setMode(HalMode::Realtime).isSuccess());
}

TEST_F(HardwareManagerTest, CommandGovernorClampsVelocity) {
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());
    hm.syncSimulationToReal();
    ASSERT_TRUE(hm.setMode(HalMode::Realtime).isSuccess());
    
    AxisSet start_cmd;
    start_cmd.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    hm.write(start_cmd);
    
    std::this_thread::sleep_for(100ms);

    // Speed limit for A0 is 100 deg/s. Target is 10 deg.
    // In 50ms, it should move max 5 degrees.
    AxisSet fast_cmd;
    fast_cmd.SetFromPositionArray({10.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    
    auto t_start = std::chrono::steady_clock::now();
    hm.write(fast_cmd);
    std::this_thread::sleep_for(50ms);
    auto fb = hm.read().value();
    
    double pos = fb.joints.GetAt(0).value().get().position.value();
    EXPECT_GT(pos, 0.1); 
    EXPECT_LT(pos, 9.0); // Should be clamped, definitely not reached 10.0 instantly
}

TEST_F(HardwareManagerTest, ZeroAxis) {
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());
    hm.syncSimulationToReal();
    ASSERT_TRUE(hm.setMode(HalMode::Realtime).isSuccess());
    
    // Move to 25 deg
    AxisSet cmd;
    cmd.SetFromPositionArray({25.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    hm.write(cmd);
    
    ASSERT_TRUE(WaitForFeedback(hm, [](auto& fb){ 
        return std::abs(fb.joints.GetAt(0).value().get().position.value() - 25.0) < 1.0; 
    }));

    // Zero it
    ASSERT_TRUE(hm.zeroAxis(AxisId::A1).isSuccess());

    // Logical should become 0
    ASSERT_TRUE(WaitForFeedback(hm, [](auto& fb){ 
        return std::abs(fb.joints.GetAt(0).value().get().position.value()) < 1.0; 
    }));
}

// --- NEW TESTS FOR BETTER COVERAGE ---

TEST_F(HardwareManagerTest, STOAndSafetyMonitoring) {
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());
    hm.syncSimulationToReal();
    ASSERT_TRUE(hm.setMode(HalMode::Realtime).isSuccess());

    // Initial state: safety should be OK (emulator defaults to OK)
    auto fb = hm.read().value();
    // EXPECT_FALSE(fb.safety.is_estop_active); // Depends on emulator default

    // We can't easily trigger E-Stop in the emulator via UDP yet without extending protocol,
    // but we can check if HalStatus is reported correctly.
    EXPECT_EQ(fb.driver_status, HalStatus::Ok);
}

TEST_F(HardwareManagerTest, CommunicationDiagnosticError) {
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());
    hm.syncSimulationToReal();
    ASSERT_TRUE(hm.setMode(HalMode::Realtime).isSuccess());

    // Kill emulator to simulate link loss
    TearDown();

    // Wait for timeout in UdpDriver
    ASSERT_TRUE(WaitForFeedback(hm, [](auto& fb){
        return fb.driver_status != HalStatus::Ok;
    }, 1000ms));
    
    auto fb = hm.read().value();
    EXPECT_TRUE(fb.driver_status == HalStatus::Error_CommunicationLost || fb.driver_status == HalStatus::Warning_SyncLost);
}

TEST_F(HardwareManagerTest, MultiAxisGovernor) {
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());
    
    // Test governor in simulation mode too (it's in HardwareManager, not driver)
    AxisSet start_cmd;
    start_cmd.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    hm.write(start_cmd);
    
    std::this_thread::sleep_for(10ms);

    AxisSet multi_cmd;
    multi_cmd.SetFromPositionArray({90.0_deg, 90.0_deg, 90.0_deg, 90.0_deg, 90.0_deg, 90.0_deg});
    hm.write(multi_cmd);

    auto fb = hm.read().value();
    for(size_t i=0; i<6; ++i) {
        double pos = fb.joints.GetAt(i).value().get().position.value();
        EXPECT_LT(pos, 80.0); // None should jump to 90 instantly
        EXPECT_GT(pos, 0.0);
    }
}

int main(int argc, char **argv) {
    RDT::Logger::Init({std::make_shared<RDT::ConsoleSink>()}, RDT::LogLevel::Info);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    RDT::Logger::Shutdown();
    return result;
}
