/**
 * @file ControllerRuntime.cpp
 * @brief Implementation of the shared controller assembly (see ControllerRuntime.h).
 *
 * The code is moved 1:1 from HexaCore's main.cpp (v0.1.19) so both editions assemble the stack
 * identically; every fail-closed policy and every startup diagnostic is preserved verbatim.
 */

#include "ControllerRuntime.h"

#include "Logger.h"
#include "RobotState.h"
#include "RdtServer.h"
#include "RobotController.h"
#include "PersistenceManager.h"
#include "kinematic_solver/KdlKinematicSolver.h"
#include "KinematicModel.h"
#include "RobotConfig.h"
#include "DataTypes.h"
#include "Units.h"

#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

namespace {
// The log tag stays "HexaCore": these are the controller-stack diagnostics operators and scripts
// already grep for ("Realtime backend:", "Programs directory:", ...), in BOTH editions.
constexpr const char* kLogTag = "HexaCore";
} // namespace

namespace RDT {

using namespace RDT::literals;

namespace {

struct RuntimeConfig {
    uint16_t rdt_server_port = 30002;
    std::string realtime_remote_ip;
    uint16_t realtime_remote_port = 30003;
    uint16_t realtime_local_port = 30004;
    std::string programs_dir = "programs";

    // Realtime backend selection:
    //   "sim"     - internal SimDriver (DEFAULT; safe, self-contained, works out of the box for debugging)
    //   "udp"     - networked UDP HAL (requires realtime_remote_ip)
    //   "mks_tcp" - MKS CAN Motor Configurator over TCP (requires mks_ip/mks_port)
    std::string realtime_interface = "sim";
    std::string mks_ip = "127.0.0.1";
    uint16_t mks_port = 30110;
};

RuntimeConfig loadRuntimeConfig(const std::string& runtime_config_path,
                                const NetProtocol::RobotConfigData& persisted_config) {
    RuntimeConfig cfg;
    cfg.realtime_remote_ip = persisted_config.ipAddress;
    // The persisted config (chosen from the HAL-backend dropdown) is the authoritative source of
    // the realtime backend. The runtime-config file below may still OVERRIDE it via an explicit
    // "realtime_interface" key (headless/CI), mirroring how it overrides realtime_remote_ip.
    if (!persisted_config.realtimeBackend.empty()) {
        cfg.realtime_interface = persisted_config.realtimeBackend;
    }

    namespace fs = std::filesystem;

    // Anchor a RELATIVE programs_dir to the deployment root - the directory holding configs/ -
    // instead of the process CWD. CWD-relative resolution silently created a SECOND, empty
    // program library (e.g. build\bin\programs) when the exe was launched from another directory,
    // and the operator "lost" every stored program with no diagnostic. The HEXAMOTION_PROGRAMS_DIR
    // override below is already absolutized, so it is never re-anchored.
    const auto anchorRelativeProgramsDir = [&runtime_config_path](RuntimeConfig& c) {
        const fs::path programs(c.programs_dir);
        if (programs.empty() || programs.is_absolute()) {
            return;
        }
        const fs::path config_dir = fs::absolute(fs::path(runtime_config_path)).parent_path();
        const fs::path deployment_root =
            (config_dir.filename() == "configs") ? config_dir.parent_path() : config_dir;
        c.programs_dir = (deployment_root / programs).lexically_normal().string();
    };

    if (!fs::exists(runtime_config_path)) {
        anchorRelativeProgramsDir(cfg);
        return cfg;
    }

    std::ifstream f(runtime_config_path);
    if (!f.is_open()) {
        Logger::Warn(kLogTag, "Runtime config exists but cannot be opened: {}. Using defaults.", runtime_config_path);
        return cfg;
    }

    try {
        nlohmann::json j;
        f >> j;

        cfg.rdt_server_port = j.value("rdt_server_port", cfg.rdt_server_port);
        cfg.realtime_remote_ip = j.value("realtime_remote_ip", cfg.realtime_remote_ip);
        cfg.realtime_remote_port = j.value("realtime_remote_port", cfg.realtime_remote_port);
        cfg.realtime_local_port = j.value("realtime_local_port", cfg.realtime_local_port);
        cfg.programs_dir = j.value("programs_dir", cfg.programs_dir);
        cfg.realtime_interface = j.value("realtime_interface", cfg.realtime_interface);
        cfg.mks_ip = j.value("mks_ip", cfg.mks_ip);
        cfg.mks_port = j.value("mks_port", cfg.mks_port);
    } catch (const std::exception& e) {
        Logger::Warn(kLogTag, "Failed to parse runtime config '{}': {}. Using defaults.", runtime_config_path, e.what());
    }

    if (const char* env_programs_dir = std::getenv("HEXAMOTION_PROGRAMS_DIR")) {
        fs::path p(env_programs_dir);
        cfg.programs_dir = fs::absolute(p).string();
    }

    anchorRelativeProgramsDir(cfg);
    return cfg;
}

// Shared candidate walk for both config files (env override first, CWD candidates, exe-relative).
std::string resolveByCandidates(int argc, char** argv, const char* env_name, const char* file_name) {
    namespace fs = std::filesystem;

    if (const char* env_cfg = std::getenv(env_name)) {
        fs::path p(env_cfg);
        if (fs::exists(p)) {
            return fs::absolute(p).string();
        }
    }

    std::vector<fs::path> candidates;
    candidates.emplace_back(fs::path("configs") / file_name);
    candidates.emplace_back(file_name);
    candidates.emplace_back(fs::path("build/bin") / file_name);
    candidates.emplace_back(fs::path("../configs") / file_name);
    candidates.emplace_back(fs::path("..") / file_name);

    if (argc > 0 && argv && argv[0]) {
        const fs::path exe_dir = fs::absolute(fs::path(argv[0])).parent_path();
        candidates.emplace_back(exe_dir / "configs" / file_name);
        candidates.emplace_back(exe_dir.parent_path() / "configs" / file_name);
        candidates.emplace_back(exe_dir / file_name);
    }

    for (const auto& c : candidates) {
        if (fs::exists(c)) {
            return fs::absolute(c).string();
        }
    }

    return (fs::path("configs") / file_name).string();
}

} // namespace

std::string ControllerRuntime::resolveConfigPath(int argc, char** argv) {
    return resolveByCandidates(argc, argv, "HEXAMOTION_CONFIG_PATH", "hexacore_config.json");
}

std::string ControllerRuntime::resolveRuntimeConfigPath(int argc, char** argv) {
    return resolveByCandidates(argc, argv, "HEXAMOTION_RUNTIME_CONFIG_PATH", "hexacore_runtime_config.json");
}

Result<std::unique_ptr<ControllerRuntime>, ErrorCode> ControllerRuntime::create(
    ControllerRuntimeOptions options) {
    // The runtime is assembled into a local instance and returned only when EVERY stage
    // succeeded (fail-fast policy of the original main preserved).
    std::unique_ptr<ControllerRuntime> rt(new ControllerRuntime());

    // 1. RobotState + controller cycle configuration.
    rt->robot_state_ = std::make_shared<RobotState>();
    ControllerConfig ctrl_config;
    ctrl_config.NrtCycleMs = std::chrono::milliseconds(4); // Match the RT loop for now
    ctrl_config.PlannerTickSec = Seconds(0.004); // 250Hz for execution
    rt->nrt_cycle_ms_ = ctrl_config.NrtCycleMs;
    rt->broadcast_ms_ = ctrl_config.BroadcastRateMs;

    // 2. Persisted configuration (fail-closed, audit TF1): an EXISTING but unusable config
    // refuses startup - running on a default config would silently change the robot frame,
    // tools and axis limits under the operator.
    Logger::Info(kLogTag, "Using config file: {}", options.config_path);
    rt->persistence_manager_ = std::make_shared<PersistenceManager>(options.config_path);
    auto config_load_result = rt->persistence_manager_->Load();
    if (config_load_result.isError()) {
        Logger::Critical(kLogTag,
                         "Configuration file '{}' exists but cannot be used ({}). REFUSING TO START. "
                         "Fix the file (see the persistence log above) or delete it to regenerate a default.",
                         options.config_path, ToString(config_load_result.error()));
        return Result<std::unique_ptr<ControllerRuntime>, ErrorCode>::Failure(config_load_result.error());
    }
    NetProtocol::RobotConfigData loaded_config = config_load_result.value();
    (void)rt->robot_state_->applyPersistedConfig(loaded_config);

    const RuntimeConfig runtime_config = loadRuntimeConfig(options.runtime_config_path, loaded_config);
    Logger::Info(kLogTag, "Using runtime config file: {}", options.runtime_config_path);

    // 3. Robot limits (sanitized) + kinematics from the persisted URDF.
    RobotLimits limits;

    auto sanitizeAxisLimitsInPlace = [](std::array<std::pair<Degrees, Degrees>, ROBOT_AXES_COUNT>& axis_limits,
                                        const char* source_tag) {
        constexpr double kFallbackMinDeg = -180.0;
        constexpr double kFallbackMaxDeg = 180.0;
        constexpr double kEps = 1e-9;

        for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            const double min_deg = axis_limits[i].first.value();
            const double max_deg = axis_limits[i].second.value();

            const bool invalid_numeric = !std::isfinite(min_deg) || !std::isfinite(max_deg);
            const bool collapsed_to_zero = (std::abs(min_deg) < kEps && std::abs(max_deg) < kEps);
            const bool invalid_order = !(min_deg < max_deg);

            if (invalid_numeric || collapsed_to_zero || invalid_order) {
                Logger::Warn(kLogTag,
                             "Axis {} limits from '{}' are invalid [{:.3f}, {:.3f}]. Applying fallback [{:.1f}, {:.1f}] deg.",
                             (i + 1), source_tag, min_deg, max_deg, kFallbackMinDeg, kFallbackMaxDeg);
                axis_limits[i] = {Degrees(kFallbackMinDeg), Degrees(kFallbackMaxDeg)};
            }
        }
    };

    const auto robot_def = rt->robot_state_->getRobotDefinition();
    limits.joint_position_limits_deg = robot_def.axis_limits;
    sanitizeAxisLimitsInPlace(limits.joint_position_limits_deg, "RobotState config");
    limits.joint_velocity_limits_deg_s.fill(1000.0_deg_s);

    const std::string urdf_path = loaded_config.urdfPath;
    RobotModelConfig model_config;
    model_config.urdf_path = urdf_path;
    model_config.base_link = "base_link";
    model_config.tip_link.clear(); // canonical tip resolution is centralized in KinematicModel
    model_config.root_transform = {
        loaded_config.modelRootX,
        loaded_config.modelRootY,
        loaded_config.modelRootZ,
        loaded_config.modelRootRxDeg,
        loaded_config.modelRootRyDeg,
        loaded_config.modelRootRzDeg
    };

    // URDF parsing is the one startup step that may surface a third-party (KDL/regex)
    // exception, so it carries a narrow boundary guard. JSON parsing is already guarded
    // inside loadRuntimeConfig() and PersistenceManager.
    std::unique_ptr<KinematicModel> model;
    try {
        model = KinematicModel::createFromURDFFile(model_config);
    } catch (const std::exception& e) {
        Logger::Critical(kLogTag, "Exception while loading URDF from '{}': {}. Startup aborted.", urdf_path, e.what());
        return Result<std::unique_ptr<ControllerRuntime>, ErrorCode>::Failure(ErrorCode::DeserializationError);
    }
    if (!model) {
        Logger::Critical(kLogTag, "URDF load failed from '{}'. Startup aborted (fail-fast policy).", urdf_path);
        return Result<std::unique_ptr<ControllerRuntime>, ErrorCode>::Failure(ErrorCode::InvalidPath);
    }
    Logger::Info(kLogTag, "Kinematic model loaded from URDF: {}", urdf_path);
    if (model->hasRobotLimits()) {
        limits = model->getRobotLimits();
        sanitizeAxisLimitsInPlace(limits.joint_position_limits_deg, "URDF");
        Logger::Info(kLogTag, "Robot limits loaded from URDF.");
    } else {
        Logger::Warn(kLogTag, "URDF loaded but limits were not fully available. Using state/default limits.");
    }
    // Keep RobotState definition synchronized with the effective (sanitized) limits.
    auto updated_def = rt->robot_state_->getRobotDefinition();
    updated_def.axis_limits = limits.joint_position_limits_deg;
    rt->robot_state_->updateRobotDefinition(updated_def);

    rt->solver_ = std::make_shared<KdlKinematicSolver>(std::move(model), limits);

    // 4. Hardware Abstraction Layer. The HAL always starts in Simulation mode (safety): even if a
    // realtime backend is configured, switching to REAL is an explicit, validated HMI action.
    InterfaceConfig hw_config;

    // Realtime backend selection is explicit and fail-safe: any unrecognized value, "sim" or
    // "none" resolves to the internal SimDriver. "udp" and "mks_tcp" are opt-in and each require
    // their endpoint; a "udp" selection without a remote IP falls back to simulation.
    if (runtime_config.realtime_interface == "mks_tcp") {
        hw_config.realtime_type = InterfaceConfig::RealtimeInterfaceType::MksTcp;
        hw_config.mks_tcp_config.ip = runtime_config.mks_ip;
        hw_config.mks_tcp_config.port = runtime_config.mks_port;

        Logger::Info(kLogTag, "Realtime backend: MKS TCP Driver {}:{}",
            hw_config.mks_tcp_config.ip,
            hw_config.mks_tcp_config.port);
    } else if (runtime_config.realtime_interface == "udp") {
        if (runtime_config.realtime_remote_ip.empty()) {
            hw_config.realtime_type = InterfaceConfig::RealtimeInterfaceType::None;
            Logger::Warn(kLogTag,
                "Realtime interface 'udp' selected but 'realtime_remote_ip' is empty. "
                "Falling back to INTERNAL SIMULATION (fail-safe).");
        } else {
            hw_config.realtime_type = InterfaceConfig::RealtimeInterfaceType::Udp;
            hw_config.udp_control_config.remote_ip = runtime_config.realtime_remote_ip;
            hw_config.udp_control_config.remote_port = runtime_config.realtime_remote_port;
            hw_config.udp_control_config.local_port = runtime_config.realtime_local_port;

            Logger::Info(kLogTag, "Realtime backend: UDP Driver {} (L:{}, R:{})",
                hw_config.udp_control_config.remote_ip,
                hw_config.udp_control_config.local_port,
                hw_config.udp_control_config.remote_port);
        }
    } else {
        // "sim", "none", empty, or any unrecognized value -> internal SimDriver.
        hw_config.realtime_type = InterfaceConfig::RealtimeInterfaceType::None;
        Logger::Info(kLogTag,
            "Realtime backend: INTERNAL SIMULATION (SimDriver). Set 'realtime_interface' to "
            "\"udp\" or \"mks_tcp\" in the runtime config to use a real backend.");
    }
    Logger::Info(kLogTag, "RDT listen: {}:{}",
                 options.rdt_transport ? std::string("in-process loopback") : std::string("0.0.0.0"),
                 runtime_config.rdt_server_port);
    Logger::Info(kLogTag, "Programs directory: {}",
                 runtime_config.programs_dir.empty() ? std::string("programs") : runtime_config.programs_dir);
    Logger::Info(kLogTag, "Resolved URDF path: {}", urdf_path);
    Logger::Info(kLogTag, "Startup mode: INTERNAL SIMULATION");

    std::array<Degrees, ROBOT_AXES_COUNT> initial_pos = {
        0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg
    };

    // Ensure both internal simulation and UDP HAL start with the same angles
    if (auto res = hw_config.simulation_initial_joints.SetFromPositionArray(initial_pos); res.isError()) {
        Logger::Warn(kLogTag, "Failed to set initial simulation pose.");
    }

    rt->hw_manager_ = std::make_shared<HardwareManager>(hw_config, limits);

    // Motion Manager (RT thread) + Trajectory Planner.
    rt->motion_manager_ = std::make_shared<MotionManager>(rt->hw_manager_, 4, limits, 5.0_deg);
    auto interpolator = std::make_shared<TrajectoryInterpolator>(rt->solver_);
    rt->planner_ = std::make_shared<TrajectoryPlanner>(interpolator, rt->motion_manager_, ctrl_config);

    // The main controller.
    rt->controller_ = std::make_shared<RobotController>(
        rt->hw_manager_, rt->motion_manager_, rt->planner_, rt->solver_, rt->robot_state_, ctrl_config
    );

    // 5. Initialize Controller.
    if (!rt->controller_->initialize()) {
        Logger::Critical(kLogTag, "Failed to initialize RobotController");
        return Result<std::unique_ptr<ControllerRuntime>, ErrorCode>::Failure(ErrorCode::InvalidState);
    }

    // 6. Network Server (bridge to the HMI) - TCP by default, injected loopback in the desktop
    // edition. The transport choice changes NOTHING above this line.
    rt->server_ = std::make_unique<RdtServer>(rt->robot_state_, rt->persistence_manager_,
                                              runtime_config.programs_dir,
                                              std::move(options.rdt_transport));
    rt->rdt_port_ = runtime_config.rdt_server_port;
    if (!rt->server_->start(rt->rdt_port_)) {
        Logger::Critical(kLogTag, "Failed to start RdtServer on port {}. Check if port is in use.", rt->rdt_port_);
        return Result<std::unique_ptr<ControllerRuntime>, ErrorCode>::Failure(ErrorCode::SocketBindFailed);
    }

    Logger::Info(kLogTag, "Controller stack assembled. Listening on port {} for HMI connections.", rt->rdt_port_);
    return Result<std::unique_ptr<ControllerRuntime>, ErrorCode>::Success(std::move(rt));
}

ControllerRuntime::~ControllerRuntime() {
    stop();
}

void ControllerRuntime::nrtLoop(std::atomic<bool>& keep_running) {
    const auto control_period = nrt_cycle_ms_;
    const auto broadcast_period = broadcast_ms_;
    auto next_broadcast_time = std::chrono::steady_clock::now() + broadcast_period;

    while (keep_running.load()) {
        auto start_time = std::chrono::steady_clock::now();

        // Run the main controller logic
        controller_->update();

        // Periodic network broadcast
        if (start_time >= next_broadcast_time) {
            server_->broadcastStatus();
            next_broadcast_time = start_time + broadcast_period;
        }

        // Sleep to maintain target frequency
        std::this_thread::sleep_until(start_time + control_period);
    }
}

void ControllerRuntime::runBlocking(std::atomic<bool>& keep_running) {
    Logger::Info(kLogTag, "Controller is running.");
    nrtLoop(keep_running);
    Logger::Info(kLogTag, "Initiating shutdown sequence...");
    stop();
}

void ControllerRuntime::start() {
    if (nrt_thread_.joinable() || stopped_.load()) {
        return;   // already running, or already shut down - start() is not restartable
    }
    internal_running_.store(true);
    nrt_thread_ = std::thread([this]() { nrtLoop(internal_running_); });
    Logger::Info(kLogTag, "Controller NRT loop started (internal thread).");
}

void ControllerRuntime::stop() {
    if (stopped_.exchange(true)) {
        return;
    }
    internal_running_.store(false);
    if (nrt_thread_.joinable()) {
        nrt_thread_.join();
    }
    if (server_) {
        server_->stop();
    }
    Logger::Info(kLogTag, "Controller runtime stopped.");
}

} // namespace RDT
