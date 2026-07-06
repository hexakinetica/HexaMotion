// KinematicSolver.h
#ifndef KINEMATIC_SOLVER_H
#define KINEMATIC_SOLVER_H

#pragma once

#include "DataTypes.h"
#include "IKError.h"

namespace RDT {

/**
 * @class KinematicSolver
 * @brief Abstract interface for performing robot kinematics (FK and IK).
 */
class KinematicSolver {
public:
    virtual ~KinematicSolver() = default;

    /**
     * @brief Computes Forward Kinematics (FK).
     * @param joints The current joint positions.
     * @param result [out] The resulting Cartesian pose.
     * @return True on success, false otherwise.
     */
    [[nodiscard]] virtual bool solveFK(const AxisSet& joints, CartPose& result) const = 0;

    /**
     * @brief Computes Inverse Kinematics (IK).
     * @param pose The target Cartesian pose.
     * @param seed_joints Initial joint positions for the iterative solver.
     * @return A Result containing the computed joint positions on success, or an IKError.
     */
    [[nodiscard]] virtual Result<AxisSet, IKError> solveIK(const CartPose& pose,
                                                           const AxisSet& seed_joints) const = 0;

    /**
     * @brief Sets the "home" position of the robot in joint space.
     */
    virtual void setHomePosition(const AxisSet& home_joints) = 0;

    /**
     * @brief Gets the current "home" position of the robot.
     */
    [[nodiscard]] virtual AxisSet getHomePosition() const = 0;
};

} // namespace RDT

#endif // KINEMATIC_SOLVER_H
