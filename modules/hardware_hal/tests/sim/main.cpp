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

class SimTest : public ::testing::Test {
protected:
    InterfaceConfig config;
    RobotLimits limits;
};

TEST_F(SimTest, InitializationAndDefaultMode) {
    std::cout << "[INFO] Testing Initialization..." << std::endl;
    HardwareManager hm(config, limits);
    auto init_res = hm.init();
    ASSERT_TRUE(init_res.isSuccess()) << "Failed to init HardwareManager";
    EXPECT_EQ(hm.getMode(), HalMode::Simulation) << "Default mode should be Simulation";
}

TEST_F(SimTest, SimulationModeReadWrite) {
    std::cout << "[INFO] Testing Read/Write in Simulation..." << std::endl;
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());
    
    AxisSet cmd;
    cmd.SetFromPositionArray({10.0_deg, 20.0_deg, 30.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    
    std::cout << "[INFO] Sending command: A1=10.0, A2=20.0..." << std::endl;
    ASSERT_TRUE(hm.write(cmd).isSuccess());
    
    auto fb_res = hm.read();
    ASSERT_TRUE(fb_res.isSuccess());
    EXPECT_NEAR(fb_res.value().joints.GetAt(0).value().get().position.value(), 10.0, 1e-6);
    EXPECT_NEAR(fb_res.value().joints.GetAt(1).value().get().position.value(), 20.0, 1e-6);
}

TEST_F(SimTest, OffsetLogicCheck) {
    std::cout << "[INFO] Testing Offset Logic in Simulation..." << std::endl;
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());

    // 1. Move to physical 10.0 (logical 10.0 initially, as offsets are 0)
    AxisSet cmd;
    cmd.SetFromPositionArray({10.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    hm.write(cmd);
    
    auto fb1 = hm.read().value();
    EXPECT_NEAR(fb1.joints.GetAt(0).value().get().position.value(), 10.0, 1e-6);

    // 2. Zero axis at this position.
    // Physical is 10.0. We want Logical to be 0.0.
    // Offset = NewLogical (0) - Physical (10) = -10.
    std::cout << "[INFO] Zeroing Axis 1 at pos=10.0..." << std::endl;
    ASSERT_TRUE(hm.zeroAxis(AxisId::A1).isSuccess());

    // 3. Read back immediately. 
    // Physical is still 10.0. Logical = Physical (10) + Offset (-10) = 0.
    auto fb2 = hm.read().value();
    EXPECT_NEAR(fb2.joints.GetAt(0).value().get().position.value(), 0.0, 1e-6) << "Logical position should be 0 after zeroing";

    // 4. Command to Logical 5.0.
    // TargetPhysical = TargetLogical (5) - Offset (-10) = 15.
    // SimDriver writes to Physical.
    std::cout << "[INFO] Moving to Logical 5.0..." << std::endl;
    cmd.SetFromPositionArray({5.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    hm.write(cmd);

    // 5. Read back.
    // Physical should be 15. Logical should be 5.
    auto fb3 = hm.read().value();
    EXPECT_NEAR(fb3.joints.GetAt(0).value().get().position.value(), 5.0, 1e-6);
}

int main(int argc, char **argv) {
    RDT::Logger::Init({}, RDT::LogLevel::Info);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
