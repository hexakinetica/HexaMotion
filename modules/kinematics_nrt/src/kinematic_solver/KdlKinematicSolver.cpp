// KdlKinematicSolver.cpp
#include "KdlKinematicSolver.h"
#include "LoggingMacros.h"

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

namespace RDT {

// Helper for conversion as Units.h seems to lack these methods
static inline double deg2rad(double deg) { return deg * UnitConstants::PI / 180.0; }
static inline double rad2deg(double rad) { return rad * 180.0 / UnitConstants::PI; }
static inline double mm2m(double mm) { return mm / 1000.0; }
static inline double m2mm(double m) { return m * 1000.0; }

// PIMPL implementation struct
struct KdlKinematicSolver::Impl {
    std::unique_ptr<KinematicModel> model;
    RobotLimits limits;
    AxisSet home_position_joints;

    // KDL Solvers
    std::unique_ptr<KDL::ChainFkSolverPos> fk_solver;
    std::unique_ptr<KDL::ChainIkSolverVel> ik_vel_solver; 
    std::unique_ptr<KDL::ChainIkSolverPos> ik_pos_solver;

    // KDL data structures
    KDL::JntArray kdl_joint_min;
    KDL::JntArray kdl_joint_max;
};

// --- Lifecycle ---
KdlKinematicSolver::KdlKinematicSolver(std::unique_ptr<KinematicModel> model, const RobotLimits& limits)
    : pimpl_(std::make_unique<Impl>()) 
{
    if (!model) {
        RDT_LOG_CRITICAL(MODULE_NAME, "KinematicModel cannot be null.");
        throw std::invalid_argument("KinematicModel cannot be null.");
    }

    pimpl_->model = std::move(model);
    pimpl_->limits = limits;
    const unsigned int dof = pimpl_->model->getNrOfJoints();
    
    if (dof != ROBOT_AXES_COUNT) {
        RDT_LOG_CRITICAL(MODULE_NAME, "Model DOF ({}) does not match required DOF ({}).", dof, ROBOT_AXES_COUNT);
        throw std::logic_error("KinematicSolver: DOF mismatch.");
    }
    
    // Convert RDT limits to KDL JntArrays
    pimpl_->kdl_joint_min.resize(dof);
    pimpl_->kdl_joint_max.resize(dof);
    for (unsigned int i = 0; i < dof; ++i) {
        pimpl_->kdl_joint_min(i) = deg2rad(limits.joint_position_limits_deg[i].first.value());
        pimpl_->kdl_joint_max(i) = deg2rad(limits.joint_position_limits_deg[i].second.value());
    }
    
    // Initialize solvers
    const auto& chain = pimpl_->model->getChain();
    pimpl_->fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
    pimpl_->ik_vel_solver = std::make_unique<KDL::ChainIkSolverVel_wdls>(chain);
    pimpl_->ik_pos_solver = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
        chain, pimpl_->kdl_joint_min, pimpl_->kdl_joint_max,
        *pimpl_->fk_solver, *pimpl_->ik_vel_solver, 300, 1e-4);

    if (!pimpl_->fk_solver || !pimpl_->ik_vel_solver || !pimpl_->ik_pos_solver) {
        RDT_LOG_CRITICAL(MODULE_NAME, "Failed to allocate KDL solvers.");
        throw std::bad_alloc();
    }

    RDT_LOG_INFO(MODULE_NAME, "KDL Solvers initialized successfully.");
}

KdlKinematicSolver::~KdlKinematicSolver() = default;
KdlKinematicSolver::KdlKinematicSolver(KdlKinematicSolver&&) noexcept = default;
KdlKinematicSolver& KdlKinematicSolver::operator=(KdlKinematicSolver&&) noexcept = default;

// --- Interface Implementation ---

bool KdlKinematicSolver::solveFK(const AxisSet& joints, CartPose& result) const {
    const auto dof = pimpl_->model->getNrOfJoints();
    if (joints.size() != dof) {
        RDT_LOG_ERROR(MODULE_NAME, "FK input AxisSet size ({}) does not match chain DOF ({}).", joints.size(), dof);
        return false;
    }
    KDL::JntArray kdl_jnt_array = toKdlJntArray(joints);
    KDL::Frame kdl_frame_result;

    if (pimpl_->fk_solver->JntToCart(kdl_jnt_array, kdl_frame_result) < 0) {
        RDT_LOG_ERROR(MODULE_NAME, "KDL FK computation failed.");
        return false;
    }
    
    result.x = Millimeters(m2mm(kdl_frame_result.p.x()));
    result.y = Millimeters(m2mm(kdl_frame_result.p.y()));
    result.z = Millimeters(m2mm(kdl_frame_result.p.z()));
    
    double kdl_r, kdl_p, kdl_y;
    kdl_frame_result.M.GetRPY(kdl_r, kdl_p, kdl_y);
    result.rx = Degrees(rad2deg(kdl_r));
    result.ry = Degrees(rad2deg(kdl_p));
    result.rz = Degrees(rad2deg(kdl_y));

    return true;
}

Result<AxisSet, IKError> KdlKinematicSolver::solveIK(const CartPose& pose,
                                                     const AxisSet& seed_joints) const {
    const auto dof = pimpl_->model->getNrOfJoints();
    if (seed_joints.size() != dof) {
        RDT_LOG_ERROR(MODULE_NAME, "IK input seed_joints size ({}) does not match chain DOF ({}).", seed_joints.size(), dof);
        return Result<AxisSet, IKError>::Failure(IKError::InternalError);
    }

    KDL::Rotation kdl_rotation = KDL::Rotation::RPY(
        deg2rad(pose.rx.value()), deg2rad(pose.ry.value()), deg2rad(pose.rz.value()));
    KDL::Vector kdl_position(
        mm2m(pose.x.value()), mm2m(pose.y.value()), mm2m(pose.z.value()));
    KDL::Frame kdl_goal_frame(kdl_rotation, kdl_position);

    KDL::JntArray kdl_seed_jnt_array = toKdlJntArray(seed_joints);
    KDL::JntArray kdl_result_jnt_array(dof);

    int ik_status = pimpl_->ik_pos_solver->CartToJnt(kdl_seed_jnt_array, kdl_goal_frame, kdl_result_jnt_array);

    if (ik_status >= 0) {
        return Result<AxisSet, IKError>::Success(fromKdlJntArray(kdl_result_jnt_array));
    } else {
        RDT_LOG_WARN(MODULE_NAME, "IK failed with KDL error code: {}. Target may be unreachable.", ik_status);
        // Map KDL error codes to our IKError enum
        switch (ik_status) {
            case KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED:
                return Result<AxisSet, IKError>::Failure(IKError::SolverTimeout);
            default:
                return Result<AxisSet, IKError>::Failure(IKError::Unreachable);
        }
    }
}

void KdlKinematicSolver::setHomePosition(const AxisSet& home_joints) {
    if (home_joints.size() != pimpl_->model->getNrOfJoints()) {
        RDT_LOG_ERROR(MODULE_NAME, "Home position size does not match model DOF.");
        return;
    }
    pimpl_->home_position_joints = home_joints;
    RDT_LOG_INFO(MODULE_NAME, "Home position set to: {}", pimpl_->home_position_joints.toJointPoseString());
}

AxisSet KdlKinematicSolver::getHomePosition() const {
    return pimpl_->home_position_joints;
}

// --- Helper Methods ---

KDL::JntArray KdlKinematicSolver::toKdlJntArray(const AxisSet& rdt_joints) const {
    const auto dof = pimpl_->model->getNrOfJoints();
    KDL::JntArray kdl_jnts(dof);
    for (unsigned int i = 0; i < dof; ++i) {
        kdl_jnts(i) = deg2rad(rdt_joints.GetAt(i).value().get().position.value());
    }
    return kdl_jnts;
}

AxisSet KdlKinematicSolver::fromKdlJntArray(const KDL::JntArray& kdl_jnts) const {
    AxisSet rdt_axes; 
    const auto dof = pimpl_->model->getNrOfJoints();
    if (kdl_jnts.rows() != dof){
        RDT_LOG_CRITICAL(MODULE_NAME, "Size mismatch converting KDL::JntArray to AxisSet.");
        throw std::length_error("KdlKinematicSolver: KDL JntArray size mismatch.");
    }
    for (unsigned int i = 0; i < dof; ++i) {
        rdt_axes.GetAt(i).value().get().position = Degrees(rad2deg(kdl_jnts(i)));
    }
    return rdt_axes;
}

} // namespace RDT
