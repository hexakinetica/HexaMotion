/**
 * @file main.cpp
 * @brief Entry point for HexaCore - Standalone Robot Controller
 * 
 * HexaCore is a migrated version of RDT-core-old with:
 * - StateData replaced by RobotState
 * - Network protocol integration via RdtServer
 * - No Qt dependencies
 * - Full compatibility with HexaStudio
 */

#include "Logger.h"
#include "RobotState.h"
#include "RdtServer.h"
#include "RobotController.h"
#include "PersistenceManager.h"
#include "KinematicModel.h"
#include "RobotConfig.h"
#include "DataTypes.h"
#include "Units.h"

#include <memory>
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

using namespace RDT;
using namespace RDT::literals;

// Global flag for graceful shutdown handling
std::atomic<bool> g_running(true);

/**
 * @brief Signal handler to initiate shutdown on Ctrl+C or termination signals.
 */
void signalHandler(int /*signum*/) {
    g_running = false;
}

int main() {
    // 1. Setup Signal Handling
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // 2. Initialize Logging
    auto console_sink = std::make_shared<ConsoleSink>();
    Logger::Init({console_sink}, LogLevel::Info);
    Logger::Info("HexaCore", "--- Starting HexaCore Robot Controller ---");

    try {
        // 3. Initialize RobotState
        auto robot_state = std::make_shared<RobotState>();

        // 4. Load initial configuration from disk
        auto persistence_manager = std::make_shared<PersistenceManager>("hexacore_config.json");
        (void)persistence_manager->LoadToRobotState(robot_state.get()); // Non-fatal on missing/invalid file

        // 5. Create RobotController
        InterfaceConfig hw_config;
        hw_config.realtime_type = InterfaceConfig::RealtimeInterfaceType::Udp;
        // Standard KUKA HOME Position matching HexaVRC
        std::array<Radians, ROBOT_AXES_COUNT> initial_positions = {
            0.0_rad, 
            -1.5708_rad, // -90 deg
            1.5708_rad,  // 90 deg
            0.0_rad, 
            1.5708_rad,  // 90 deg
            0.0_rad
        };
        auto result = hw_config.simulation_initial_joints.SetFromPositionArray(initial_positions);
        if (result.isError()) {
            Logger::Error("HexaCore", "Failed to set initial joint positions");
            return 1;
        }

        ControllerConfig ctrl_config;
        KinematicModel kuka_model = KinematicModel::createKR6R900();
        
        auto controller = std::make_shared<RobotController>(
            hw_config, 
            ctrl_config, 
            kuka_model, 
            robot_state
        );

    // 6. Initialize Controller with initial state
    TrajectoryPoint initial_state;
    initial_state.command.joint_target = hw_config.simulation_initial_joints;
    initial_state.header.tool = ToolFrame("DefaultTool", CartPose{});
    initial_state.header.base = BaseFrame("DefaultBase", CartPose{});
    
    // ВАЖНО: Инициализируем CommandTrajectoryPoint до запуска цикла
    robot_state->updateCommandTrajectoryPoint(initial_state);

    if (!controller->initialize(initial_state)) {
            Logger::Critical("HexaCore", "Failed to initialize RobotController");
            return 1;
        }

        // 7. Initialize Network Server (Bridge to HexaStudio)
        RdtServer server(robot_state, persistence_manager);
        uint16_t port = 30002;
        if (!server.start(port)) {
            Logger::Critical("HexaCore", "Failed to start RdtServer on port {}. Check if port is in use.", port);
            return 1;
        }

        Logger::Info("HexaCore", "Controller is running. Press Ctrl+C to terminate.");
        Logger::Info("HexaCore", "Listening on port {} for HexaStudio connections.", port);

    // 8. Main Control and Communication Loop
    constexpr auto control_period = std::chrono::microseconds(4000); // 250 Hz control loop (4ms)
    constexpr int network_divider = 25; // Broadcast status every 25 cycles (10 Hz)
        int cycle_count = 0;

        while (g_running) {
            auto start_time = std::chrono::steady_clock::now();

            // Process network commands (RobotController checks RobotState internally)
            // The RdtServer already calls robot_state->processNetworkCommand() when receiving data
            // We just need to let RobotController react to state changes
            controller->processNetworkCommands();

            // Periodic network broadcast
            if (++cycle_count >= network_divider) {
                server.broadcastStatus();
                cycle_count = 0;
            }
            
            // Sleep to maintain target frequency
            std::this_thread::sleep_until(start_time + control_period);
        }

        Logger::Info("HexaCore", "Initiating shutdown sequence...");
        server.stop();
        // Controller will be destroyed automatically, calling destructor

    } catch (const std::exception& e) {
        Logger::Critical("HexaCore", "A fatal error occurred: {}", e.what());
        return 1;
    }

    Logger::Info("HexaCore", "Shutdown complete. Goodbye.");
    Logger::Shutdown();
    return 0;
}
