/**
 * @file ControllerRuntime.h
 * @brief The ONE controller assembly shared by both product editions.
 *
 * Extracted from HexaCore's main.cpp (boss directive: the desktop edition must reuse the same
 * modules with zero duplication - the assembly logic is exactly what would otherwise drift).
 * Consumers:
 *   - HexaCore.exe (networked edition): create() + runBlocking() on the console main thread,
 *     RDT over TCP (default transport);
 *   - HexaStudioDesktop.exe (single-process edition): create() with an injected loopback
 *     transport + start()/stop() around the Qt event loop.
 *
 * The runtime owns the full controller stack: RobotState, persistence, URDF/KDL kinematics,
 * HardwareManager (HAL), MotionManager (RT thread), TrajectoryPlanner, RobotController and the
 * RdtServer. Fail-closed policies from the original main are preserved verbatim (unusable config
 * refuses startup, URDF load is fail-fast, HAL starts in Simulation).
 */

#ifndef HEXA_CONTROLLER_RUNTIME_H
#define HEXA_CONTROLLER_RUNTIME_H

#pragma once

#include "ErrorCode.h"
#include "result.h"
#include "INetworkServer.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace RDT {

class RobotState;
class PersistenceManager;
class HardwareManager;
class MotionManager;
class TrajectoryPlanner;
class KdlKinematicSolver;
class RobotController;
class RdtServer;

/**
 * @struct ControllerRuntimeOptions
 * @brief Everything create() needs; both paths come from the resolve helpers below.
 */
struct ControllerRuntimeOptions {
    std::string config_path;           ///< persisted robot config (resolveConfigPath)
    std::string runtime_config_path;   ///< runtime config: ports, backend, programs dir
    /// Optional injected RDT transport. nullptr = platform TCP server (networked edition).
    /// The desktop edition injects a LoopbackNetworkServer bound to its in-process hub.
    std::unique_ptr<INetworkServer> rdt_transport;
};

class ControllerRuntime {
public:
    /**
     * @brief Resolves the persisted-config path: HEXAMOTION_CONFIG_PATH env override, then the
     * documented CWD/exe-relative candidates (moved 1:1 from HexaCore main).
     */
    [[nodiscard]] static std::string resolveConfigPath(int argc, char** argv);
    /** @brief Same resolution scheme for the runtime config (HEXAMOTION_RUNTIME_CONFIG_PATH). */
    [[nodiscard]] static std::string resolveRuntimeConfigPath(int argc, char** argv);

    /**
     * @brief Builds and starts the full controller stack (including the RdtServer).
     * All startup diagnostics are logged here, identically for both editions.
     * Fail-closed: an unusable persisted config or a failed URDF/controller/server init refuses
     * startup with a typed error; nothing runs in a half-initialized state.
     */
    [[nodiscard]] static Result<std::unique_ptr<ControllerRuntime>, ErrorCode> create(
        ControllerRuntimeOptions options);

    ~ControllerRuntime();

    ControllerRuntime(const ControllerRuntime&) = delete;
    ControllerRuntime& operator=(const ControllerRuntime&) = delete;

    /**
     * @brief Runs the NRT control loop on the CALLING thread until keep_running goes false,
     * then shuts the server down. Used by the HexaCore console main (Ctrl+C flips the flag).
     */
    void runBlocking(std::atomic<bool>& keep_running);

    /** @brief Desktop edition: runs the NRT loop on an internal thread. Idempotent. */
    void start();
    /** @brief Stops the NRT loop (if start() was used) and the RdtServer. Idempotent. */
    void stop();

    /** @brief The RDT port the server listens on (loopback editions pass it to the client side). */
    [[nodiscard]] uint16_t rdtPort() const { return rdt_port_; }

private:
    ControllerRuntime() = default;

    void nrtLoop(std::atomic<bool>& keep_running);

    std::shared_ptr<RobotState> robot_state_;
    std::shared_ptr<PersistenceManager> persistence_manager_;
    std::shared_ptr<KdlKinematicSolver> solver_;
    std::shared_ptr<HardwareManager> hw_manager_;
    std::shared_ptr<MotionManager> motion_manager_;
    std::shared_ptr<TrajectoryPlanner> planner_;
    std::shared_ptr<RobotController> controller_;
    std::unique_ptr<RdtServer> server_;

    uint16_t rdt_port_ = 0;
    // Cycle timing captured from the controller config at assembly time.
    std::chrono::milliseconds nrt_cycle_ms_{4};
    std::chrono::milliseconds broadcast_ms_{100};

    // start()/stop() thread machinery (desktop edition).
    std::thread nrt_thread_;
    std::atomic<bool> internal_running_{false};
    std::atomic<bool> stopped_{false};
};

} // namespace RDT

#endif // HEXA_CONTROLLER_RUNTIME_H
