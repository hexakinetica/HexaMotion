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

namespace {
HardwareFeedback readLatest(HardwareManager& hm) {
    // Sim driver updates internal state after read; call twice to get latest
    auto first = hm.read();
    auto second = hm.read();
    return second.isSuccess() ? second.value() : first.value();
}
}

class SimTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Valid limits: SimDriver seeds its HAL soft/velocity limits from RobotLimits
        // (REQ-SIMMC-09); the bench must ship a valid config, not default zeros.
        limits.joint_position_limits_deg.fill({-180.0_deg, 180.0_deg});
        limits.joint_velocity_limits_deg_s.fill(1000.0_deg_s);
    }

    InterfaceConfig config;
    RobotLimits limits;
};

TEST_F(SimTest, InitializationAndDefaultMode) {
    HardwareManager hm(config, limits);
    auto init_res = hm.init();
    ASSERT_TRUE(init_res.isSuccess());
    EXPECT_EQ(hm.getMode(), HalMode::Simulation);
}

TEST_F(SimTest, OffsetLogicCheck) {
    // Sim backend = virtual Motor Configurator in the real-driver slot (REQ-SIMMC-01). Zeroing
    // rides the production real-first routing and therefore targets the virtual MC, NOT the
    // active sim robot (REQ-SIMMC-07) — same referent as with real hardware.
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());

    // 1. Move the ACTIVE sim robot to logical 10.0 — it must stay untouched by calibration.
    AxisSet cmd;
    cmd.SetFromPositionArray({10.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    hm.write(cmd);

    auto fb1 = readLatest(hm);
    EXPECT_NEAR(fb1.joints.GetAt(0).value().get().position.value(), 10.0, 1e-6);

    // 2. Jog the VIRTUAL MC to 10.0 through the production HAL-jog path, then zero it there.
    // read() in Simulation mode refreshes the real-feedback cache from the virtual MC.
    ASSERT_TRUE(hm.jogRealIncremental(0, 10.0, 1.0).isSuccess());
    (void)hm.read();
    auto mc_before = hm.getRealDriverFeedback();
    ASSERT_TRUE(mc_before.isSuccess());
    EXPECT_NEAR(mc_before.value().joints.GetAt(0).value().get().position.value(), 10.0, 1e-6);

    ASSERT_TRUE(hm.zeroAxis(AxisId::A1).isSuccess());

    // 3. The virtual MC now reads logical 0.0 at the jogged physical position.
    (void)hm.read();
    auto mc_after = hm.getRealDriverFeedback();
    ASSERT_TRUE(mc_after.isSuccess());
    EXPECT_NEAR(mc_after.value().joints.GetAt(0).value().get().position.value(), 0.0, 1e-6);

    // 4. The ACTIVE sim robot is untouched by the calibration.
    auto fb2 = readLatest(hm);
    EXPECT_NEAR(fb2.joints.GetAt(0).value().get().position.value(), 10.0, 1e-6);
}

TEST_F(SimTest, MasterAxisAtSpecificPosition) {
    // Mastering rides the production real-first routing: the calibration target is the virtual
    // MC (REQ-SIMMC-07), read back through getRealDriverFeedback (the ghost source).
    HardwareManager hm(config, limits);
    ASSERT_TRUE(hm.init().isSuccess());

    // 1. The virtual MC is physically at 0.0 after init.
    (void)hm.read();
    auto mc_init = hm.getRealDriverFeedback();
    ASSERT_TRUE(mc_init.isSuccess());
    EXPECT_NEAR(mc_init.value().joints.GetAt(1).value().get().position.value(), 0.0, 1e-6);

    // 2. We say: "This position corresponds to logical 90.0 degrees"
    // (e.g. robot is horizontal, but encoder says 0)
    ASSERT_TRUE(hm.masterAxisAt(AxisId::A2, 90.0_deg).isSuccess());

    // 3. Read back from the virtual MC. Logical should be 90.0.
    (void)hm.read();
    auto mc_new = hm.getRealDriverFeedback();
    ASSERT_TRUE(mc_new.isSuccess());
    EXPECT_NEAR(mc_new.value().joints.GetAt(1).value().get().position.value(), 90.0, 1e-6);

    // 4. The ACTIVE sim robot keeps its own offsets (untouched by MC mastering): commanding
    // logical 95.0 reads back 95.0.
    AxisSet cmd;
    cmd.SetFromPositionArray({0.0_deg, 95.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    hm.write(cmd);

    auto fb_final = readLatest(hm);
    EXPECT_NEAR(fb_final.joints.GetAt(1).value().get().position.value(), 95.0, 1e-6);
}

// Note: the Sim<->Real pose-mismatch rejection of setMode(Realtime) is NOT tested here.
// It requires a realtime backend reporting HalStatus::Ok, which cannot exist in this
// emulator-less suite. The scenario is covered by RealtimeTest.ModeSwitchFailAndSync in
// tests/realtime, which runs the emulator. What IS testable without an emulator is the
// fail-closed refusal below.

TEST_F(SimTest, ModeSwitchWithoutLinkIsRejected) {
    InterfaceConfig realtime_config;
    realtime_config.realtime_type = InterfaceConfig::RealtimeInterfaceType::Udp;
    realtime_config.udp_control_config.remote_ip = "127.0.0.1";
    realtime_config.udp_control_config.remote_port = 50001;
    realtime_config.udp_control_config.local_port = 50002;
    realtime_config.debug_stream_config.enabled = false;

    HardwareManager hm(realtime_config, limits);
    ASSERT_TRUE(hm.init().isSuccess());

    // No emulator is running: the UDP link never reports Ok, so the switch to REAL must be
    // refused fail-closed instead of silently entering Realtime with a dead link (which would
    // also skip the pose-sync gate).
    auto switch_res = hm.setMode(HalMode::Realtime);
    ASSERT_TRUE(switch_res.isError()) << "Switch to REAL with a dead link must be refused";
    EXPECT_EQ(switch_res.error(), ErrorCode::NotConnected);
    EXPECT_EQ(hm.getMode(), HalMode::Simulation);
}

TEST_F(SimTest, SetHalConfigOnSimOnlyBenchReportsOk) {
    HardwareManager hm(config, limits); // realtime_type == None (sim-only bench)
    ASSERT_TRUE(hm.init().isSuccess());

    HalConfigCommand cmd;
    cmd.requestId = 7;
    for (auto& axis : cmd.axes) {
        axis.update_mask = HalConfigUpdate_MotorEnabled;
        axis.motor_enabled = true;
    }
    ASSERT_TRUE(hm.setHalConfig(cmd).isSuccess());

    // No realtime backend is configured, so its absence is not a communication error: the
    // committed state must reflect the successful simulation apply, not CommunicationLost.
    const auto state = hm.getCurrentHalState();
    EXPECT_EQ(state.appliedRequestId, 7u);
    for (const auto& axis : state.axes) {
        EXPECT_EQ(axis.last_status, static_cast<int>(HalStatus::Ok));
        EXPECT_EQ(axis.last_error_code, 0);
        EXPECT_TRUE(axis.motor_enabled);
    }
}

int main(int argc, char **argv) {
    RDT::Logger::Init({std::make_shared<RDT::ConsoleSink>()}, RDT::LogLevel::Info);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    RDT::Logger::Shutdown();
    return result;
}