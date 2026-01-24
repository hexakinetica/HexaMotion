// KdlKinematicSolver.h
#ifndef KDL_KINEMATIC_SOLVER_H
#define KDL_KINEMATIC_SOLVER_H

#pragma once

#include "KinematicSolver.h"
#include "KinematicModel.h"
#include "RobotConfig.h" // For RobotLimits
#include "DataTypes.h"
#include "Units.h"
#include "IKError.h"

#include <kdl/chain.hpp>
#include <memory>
#include <atomic>

// Forward-declare KDL types to hide them from the public interface (PIMPL)
namespace KDL {
    class ChainFkSolverPos;
    class ChainIkSolverVel;
    class ChainIkSolverPos;
    class JntArray;
}

namespace RDT {

/**
 * @class KdlKinematicSolver
 * @brief A concrete implementation of the KinematicSolver interface using the KDL library.
 *
 * This class provides Forward and Inverse Kinematics solutions for a given KinematicModel
 * and RobotLimits. It uses a combination of KDL solvers for robustness and performance,
 * including a weighted DLS solver for velocity and a Newton-Raphson solver for position.
 *
 * This class uses the PIMPL idiom to hide KDL implementation details.
 * @version 2.0 (Refacored)
 */
class KdlKinematicSolver : public KinematicSolver {
public:
    /**
     * @brief Constructs the KDL solver.
     * @param model A unique_ptr to the robot's geometric model. The solver takes ownership.
     * @param limits The robot's physical limits (position, velocity).
     */
    KdlKinematicSolver(std::unique_ptr<KinematicModel> model, const RobotLimits& limits);
    ~KdlKinematicSolver() override;

    // Solver owns a unique_ptr, so it should be non-copyable but movable.
    KdlKinematicSolver(const KdlKinematicSolver&) = delete;
    KdlKinematicSolver& operator=(const KdlKinematicSolver&) = delete;
    KdlKinematicSolver(KdlKinematicSolver&&) noexcept;
    KdlKinematicSolver& operator=(KdlKinematicSolver&&) noexcept;

    // --- Interface Implementation ---
    [[nodiscard]] bool solveFK(const AxisSet& joints, CartPose& result) const override;
    
    [[nodiscard]] Result<AxisSet, IKError> solveIK(const CartPose& pose,
                                                   const AxisSet& seed_joints) const override;

    void setHomePosition(const AxisSet& home_joints) override;
    [[nodiscard]] AxisSet getHomePosition() const override;

private:
    // PIMPL: Pointer to implementation
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    
    // --- Helper Methods ---
    [[nodiscard]] KDL::JntArray toKdlJntArray(const AxisSet& rdt_joints) const;
    [[nodiscard]] AxisSet fromKdlJntArray(const KDL::JntArray& kdl_jnts) const;

    static inline const std::string MODULE_NAME = "KdlSolver"; 
};

} // namespace RDT

#endif // KDL_KINEMATIC_SOLVER_H
