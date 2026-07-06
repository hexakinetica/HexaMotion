// --- START OF FILE: HexaMotion/main.cpp ---
/**
 * @file main.cpp
 * @brief Entry point for HexaCore - Standalone Robot Controller (networked edition).
 *
 * Composition-root duties ONLY: version banner, signal handling, logging, host diagnostics.
 * The controller stack itself is assembled by the shared ControllerRuntime (app/ControllerRuntime.h),
 * which the HexaStudioDesktop edition reuses verbatim - one assembly, two products.
 */

#include "ControllerRuntime.h"
#include "Logger.h"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

using namespace RDT;

// HexaCore version. Increment on every behavioral change (project rule). Version tracking
// starts here, with the HAL facade simplification batch.
// 0.1.1: CIRC circular motion (MoveC) executable end-to-end (docs/REQ_motion_circ.md).
// 0.1.2: SPLINE blocks (MoveS runs) executable end-to-end; preview renders CIRC + SPLINE
//        (docs/REQ_motion_spline.md, batches 2a+2b).
// 0.1.3: SET DO executable (sequencer P3): IDriver::setDigitalOutput (fail-closed default), SIM
//        DO1..32 + DI loopback, MKS forward-compatible IPC; controller actuates SetOutput actions
//        and faults the program if the backend refuses (docs/REQ_program_sequencer.md).
// 0.1.4: TrajectoryPoint cleanup batch B: RobotCommandFrame.cartesian_valid replaces the (0,0,0)
//        magic-zero sentinel; Cartesian segments refuse an invalid start pose (fail-closed);
//        motion chains continue from the last RENDERED point (docs/REQ_trajectorypoint_cleanup_batchB.md).
// 0.1.5: HAL hardening batch — setMode(Realtime) fail-closed on a not-Ok link; UDP worker
//        survives receive errors (link recovers without reinit); sim-only bench no longer
//        stamps CommunicationLost on successful config apply; RT path refreshes the HAL axis
//        mirror as a POD slice via IDriver::getAxisRuntimeState (no std::string copies per cycle).
// 0.1.6: sequencer P4 — integer register file (R[0..15], SetVar/IncVar/DecVar), register-compare
//        IF (counter loops run exactly N iterations), BREAK = immediate program stop from code;
//        RDT protocol v2, program file format v2 (docs/REQ_program_sequencer.md).
// 0.1.7: sequencer P5 — execution annotation rides the status loop: ProgramState carries the live
//        register snapshot + last evaluated IF (line, taken/not); published after every sequencer
//        advance and cleared on a fresh RUN (docs/REQ_program_sequencer.md §9).
// 0.1.8: TrajectoryPoint cleanup batches C+D — stage-ownership matrix documented
//        (TrajectoryPoint_stage_ownership.md); RT command buffer std::deque -> fixed RtPointRing
//        (measured 1.0 heap alloc per RT cycle at 800 B/point vs the 512 B deque block; ring = zero
//        heap traffic on the RT thread, fail-visible full guard).
// 0.1.9: STOP/PAUSE hold replan planned in joint space — overrideTrajectory() forces the hold
//        waypoint to a zero-length JOINT segment; feedback points carry HOLD (idle RT buffer) or
//        SPLINE/CIRC (mid-move), none plannable as a single segment, so STOP while standing failed
//        with UnsupportedMotionType (planner error 3) and planned no hold segment (REQ-PLAN-06).
// 0.1.10: P5 audit fix — handleProgramUpdateRequest publishes the cleared execution annotation on a
//         successful load, so the pendant's register/branch strip no longer shows the PREVIOUS
//         program's state against freshly loaded lines (parallels startProgram's reset publish).
// 0.1.14: default realtime backend is now INTERNAL SIMULATION ("sim") for out-of-the-box debugging -
//         no MKS/UDP endpoint needed to start. realtime_interface selection made explicit and
//         fail-safe: "udp"/"mks_tcp" are opt-in (udp without a remote IP falls back to sim), and any
//         unknown/"sim"/"none"/empty value resolves to the SimDriver. Runtime config default flipped
//         mks_tcp -> sim (configs/hexacore_runtime_config.json).
// 0.1.15: realtime backend is now selectable from the Settings HAL-backend dropdown in both editions.
//         The persisted config (RobotConfigData.realtimeBackend) is the authoritative source, applied
//         at startup; the runtime-config "realtime_interface" key is now an optional headless override
//         (dropped from the shipped default). Applied on controller (re)start, not hot-switched
//         (docs/REQ_hexastudio_desktop.md).
// 0.1.16: HAL backend selection made LIVE and honest for the HAL panel (boss directive 2026-07-06):
//         sim <-> mks_tcp switch at runtime via HardwareManager::setRealtimeInterfaceType (fail-closed
//         while REAL mode is active; udp stays startup-only for now). Sim backend: HAL-panel jog
//         drives the internal SimDriver directly (was NotConnected), simulated motors are ENABLED
//         out of the box. RobotController watches RobotConfigData.realtimeBackend and applies a
//         changed selection exactly once (docs/REQ_hexastudio_desktop.md §3).
// 0.1.17: sim-backend REAL mode fix — with realtime_type None the SIM->REAL switch previously failed
//         ("Real Driver Init Failed"), latched hasError() and BLOCKED all subsequent commands
//         (including HAL axis enable) until CLEAR ERRORS. Root fix: with the sim backend the
//         built-in simulator IS the hardware — setMode(Realtime) succeeds on it, syncSimulationToReal
//         is a no-op, getActiveDriverInstance falls back to the sim driver in REAL mode (UDP/MKS keep
//         the honest null-driver fault). REQ-HAL-10.
// 0.1.18: relative programs_dir is anchored to the deployment root (the directory holding configs/)
//         instead of the process CWD. Launching HexaCore.exe from another directory silently created
//         a second, EMPTY program library (e.g. build\bin\programs) and the operator "lost" the
//         stored programs without a diagnostic. HEXAMOTION_PROGRAMS_DIR override unchanged (already
//         absolute); the startup log now shows the absolute path.
// 0.1.19: sim-backend service commands behave adequately (boss report 2026-07-07): HOMING moves the
//         simulated axes to logical zero (SimDriver::requestHoming implemented; was a typed refusal);
//         SET ZERO / SET ZERO ALL take effect on the simulator - after either service op the RT hold
//         is resynced via MotionManager::reset() so the 4 ms hold stream adopts the new pose instead
//         of snapping the axes back. Error-state drops are no longer silent: HOME/SET ZERO arriving
//         while ERROR is latched are consumed with an explicit WARN + operator message
//         ("clear the ERROR first") instead of vanishing without a trace.
// 0.1.20: controller assembly extracted into the shared ControllerRuntime (app/ControllerRuntime.h,
//         moved 1:1 from this file) so the HexaStudioDesktop single-process edition reuses the
//         exact same stack; RdtServer accepts an optional injected transport (loopback). This main
//         is now a thin composition root: banner, signals, logging, host diagnostics, runBlocking.
//         Behavior of the networked edition is unchanged (docs/REQ_hexastudio_desktop.md).
// 0.1.21: dead-code audit (boss directive 2026-07-07, no behavioral change): orphan files deleted
//         (DummyKinematicSolver.cpp, utils/udp_sim/ - zero references in any CMake/test/production
//         path; UDP HAL emulation is the standalone HexaHAL_Client); controller_test_app now links
//         hexacore_runtime instead of recompiling RobotController/ProgramSequencer a second time;
//         dead commented RdtBridgeTestApp block removed from rdt_bridge CMake; demo runner
//         (run_motion_demo.py) updated to the wire-format v2 demo envelope.
// 0.1.22: sim backend = virtual Motor Configurator (docs/REQ_sim_backend_virtual_mc.md, boss
//         directive 2026-07-07: controller must not feel a difference vs TCP hardware). A second
//         SimDriver instance is installed as the realtime driver on the sim backend, so the ghost,
//         MC link status, SIM<->REAL pose gate and service commands run the SAME code as mks_tcp;
//         keep-alive Hold is not sent to the virtual MC (it follows streams, unlike MksTcpDriver);
//         SimDriver executes segment targets (instant arrival, sim-homing convention) - the
//         IDriver fallback would have streamed all-zeros; HAL-panel jog drives the virtual MC in
//         SIM view (planner path kept in REAL view only, where the virtual MC is stream-followed).
// 0.1.23: following-error regression fix (frequent Error_FollowingError on 100 mm moves,
//         introduced in 0.1.22): SimDriver::writeCommand no longer consumes segment targets —
//         MotionManager tags EVERY streamed point with segment metadata (for the MKS firmware),
//         so the segment-consuming SimDriver teleported the active robot to the segment endpoint
//         mid-move and the streamed setpoint vs actual delta tripped the 20 deg threshold. The
//         virtual-MC HAL jog now rides SimDriver::setState (REQ-SIMMC-05 1.1.0).
// 0.1.24: REAL-view Safety Error right after program start fixed (REQ-SIMMC-09): SimDriver's HAL
//         velocity/soft limits are now seeded from the ROBOT limits instead of a 100 deg/s
//         hardcode — the command governor (REAL view only) clamped the stream to 100 deg/s while
//         the planner profiled up to the robot limit (1000 deg/s default; the HexaArmMedium URDF
//         has no <limit> tags), so the actual lagged the command and Error_FollowingError (5 deg)
//         tripped. The governor now logs a one-shot warning per motion burst when it clamps.
constexpr const char* kHexaCoreVersion = "0.1.24";

// Global flag for graceful shutdown handling
std::atomic<bool> g_running(true);

/**
 * @brief Signal handler to initiate shutdown on Ctrl+C or termination signals.
 */
void signalHandler(int /*signum*/) {
    g_running = false;
}

namespace {

std::string resolveLogPath(int argc, char** argv,
                           const char* env_name,
                           const char* file_name) {
    namespace fs = std::filesystem;

    if (const char* env_path = std::getenv(env_name)) {
        fs::path p(env_path);
        if (p.has_parent_path()) {
            std::error_code ec;
            fs::create_directories(p.parent_path(), ec);
        }
        return p.string();
    }

    fs::path base_dir = fs::current_path();
    if (argc > 0 && argv && argv[0]) {
        const fs::path exe_dir = fs::absolute(fs::path(argv[0])).parent_path();
        const std::string exe_dir_name = exe_dir.filename().string();
        if (exe_dir_name == "bin" || exe_dir_name == "releases") {
            base_dir = exe_dir.parent_path();
        } else {
            base_dir = exe_dir;
        }
    }

    const fs::path logs_dir = base_dir / "logs";
    std::error_code ec;
    fs::create_directories(logs_dir, ec);
    return (logs_dir / file_name).string();
}

std::vector<std::string> getLocalIpv4Addresses() {
    std::vector<std::string> ips;

#ifdef _WIN32
    WSADATA wsaData;
    const bool wsa_ok = (WSAStartup(MAKEWORD(2, 2), &wsaData) == 0);
    if (!wsa_ok) {
        return ips;
    }
#endif

    char hostname[256]{};
    if (gethostname(hostname, sizeof(hostname)) != 0) {
#ifdef _WIN32
        WSACleanup();
#endif
        return ips;
    }

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo* result = nullptr;
    if (getaddrinfo(hostname, nullptr, &hints, &result) == 0 && result) {
        std::set<std::string> unique_ips;
        for (addrinfo* p = result; p != nullptr; p = p->ai_next) {
            if (!p->ai_addr) continue;
            auto* ipv4 = reinterpret_cast<sockaddr_in*>(p->ai_addr);
            char ip_buf[INET_ADDRSTRLEN]{};
            if (inet_ntop(AF_INET, &(ipv4->sin_addr), ip_buf, INET_ADDRSTRLEN)) {
                unique_ips.insert(std::string(ip_buf));
            }
        }
        freeaddrinfo(result);
        ips.assign(unique_ips.begin(), unique_ips.end());
    }

#ifdef _WIN32
    WSACleanup();
#endif

    return ips;
}

} // namespace

int main(int argc, char** argv) {
    // 1. Setup Signal Handling
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // 2. Initialize Logging
    auto console_sink = std::make_shared<ConsoleSink>();
    auto file_sink = std::make_shared<FileSink>(resolveLogPath(argc, argv, "HEXACORE_LOG_PATH", "hexacore_debug.log"));
    Logger::Init({console_sink, file_sink}, LogLevel::Debug);
    Logger::Info("HexaCore", "--- Starting HexaCore Robot Controller v{} ---", kHexaCoreVersion);

    // 3. Host diagnostics (operator hint for the pendant's endpoint setting).
    const char* host_name_env = std::getenv(
#ifdef _WIN32
        "COMPUTERNAME"
#else
        "HOSTNAME"
#endif
    );
    const std::string host_name = (host_name_env && *host_name_env) ? std::string(host_name_env) : std::string("unknown-host");
    Logger::Info("HexaCore", "Host: {}", host_name);

    const auto local_ips = getLocalIpv4Addresses();
    if (local_ips.empty()) {
        Logger::Warn("HexaCore", "No local IPv4 addresses detected for operator hint.");
    } else {
        for (const auto& ip : local_ips) {
            Logger::Info("HexaCore", "Local IPv4: {}", ip);
        }
    }

    // 4. Assemble and run the shared controller stack (app/ControllerRuntime.h). Default TCP
    // transport: this is the networked edition. All fail-closed policies live in the runtime.
    ControllerRuntimeOptions options;
    options.config_path = ControllerRuntime::resolveConfigPath(argc, argv);
    options.runtime_config_path = ControllerRuntime::resolveRuntimeConfigPath(argc, argv);

    auto runtime_result = ControllerRuntime::create(std::move(options));
    if (runtime_result.isError()) {
        Logger::Critical("HexaCore", "Controller startup refused: {}.", ToString(runtime_result.error()));
        Logger::Shutdown();
        return 1;
    }
    auto runtime = std::move(runtime_result).value();

    Logger::Info("HexaCore", "Controller is running. Press Ctrl+C to terminate.");
    runtime->runBlocking(g_running);

    Logger::Info("HexaCore", "Shutdown complete. Goodbye.");
    Logger::Shutdown();
    return 0;
}
// --- END OF FILE: HexaMotion/main.cpp ---
