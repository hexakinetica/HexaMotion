#include "gtest/gtest.h"
#include "HardwareManager.h"
#include "RobotConfig.h"
#include "Logger.h"
#include "LoggingMacros.h"
#include "Units.h"

#include <thread>
#include <chrono>
#include <iostream>

using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;

class GovernorTest : public ::testing::Test {
protected:
    void SetUp() override {
        limits.joint_velocity_limits_deg_s.fill(1000.0_deg_s);
        limits.joint_velocity_limits_deg_s[0] = 10.0_deg_s; // Reduce limit to 10 deg/s
    }

    InterfaceConfig config;
    RobotLimits limits;
};

TEST_F(GovernorTest, CommandGovernorClampsVelocity) {
    std::cout << "[INFO] Testing Positive Velocity Clamping..." << std::endl;
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());
    
    AxisSet start_cmd;
    start_cmd.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    hm.write(start_cmd);
    
    // Cycle time 200ms
    std::this_thread::sleep_for(200ms);

    // Limit is 10 deg/s. In 200ms, max move is 10 * 0.2 = 2.0 deg.
    // Target is 10.0 deg.
    // Result should be 2.0 deg.
    AxisSet fast_cmd;
    fast_cmd.SetFromPositionArray({10.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    hm.write(fast_cmd);
    
    auto fb = hm.read().value();
    double pos = fb.joints.GetAt(0).value().get().position.value();
    
    // Allow slight tolerance for timing jitter, but it should be close to 2.0, not 10.0
    EXPECT_NEAR(pos, 2.0, 0.5); 
}

TEST_F(GovernorTest, CommandGovernorClampsNegativeVelocity) {
    std::cout << "[INFO] Testing Negative Velocity Clamping..." << std::endl;
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());
    
    // Start at 0
    AxisSet start_cmd;
    start_cmd.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    hm.write(start_cmd);
    std::this_thread::sleep_for(200ms);

    // Command -10 deg. Limit 10 deg/s.
    // In 200ms, max move is -2.0 deg.
    AxisSet neg_cmd;
    neg_cmd.SetFromPositionArray({-10.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    hm.write(neg_cmd);

    auto fb = hm.read().value();
    double pos = fb.joints.GetAt(0).value().get().position.value();
    
    EXPECT_NEAR(pos, -2.0, 0.5);
}

TEST_F(GovernorTest, MultiAxisGovernor) {
    std::cout << "[INFO] Testing Multi-Axis Governor..." << std::endl;
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());
    
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
        EXPECT_LT(pos, 80.0);
        EXPECT_GT(pos, 0.0);
    }
}

int main(int argc, char **argv) {
    RDT::Logger::Init({}, RDT::LogLevel::Info);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
