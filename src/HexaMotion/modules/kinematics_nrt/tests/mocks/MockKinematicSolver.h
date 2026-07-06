// MockKinematicSolver.h
#ifndef MOCK_KINEMATIC_SOLVER_H
#define MOCK_KINEMATIC_SOLVER_H

#pragma once

#include "kinematic_solver/KinematicSolver.h"
#include "gmock/gmock.h"

namespace RDT {

/**
 * @class MockKinematicSolver
 * @brief A Google Mock implementation of the KinematicSolver interface for testing.
 */
class MockKinematicSolver : public KinematicSolver {
public:
    // MOCK_METHOD is a GMock macro to create a mock implementation of a virtual function.
    
    // For solveFK, we use MOCK_METHOD directly as its signature is simple.
    // Note: Since solveFK uses a non-const reference for an output parameter,
    // we need to tell GMock about it. A simpler interface would be to return a std::optional<CartPose>.
    MOCK_METHOD(bool, solveFK, (const AxisSet& joints, CartPose& result), (const, override));

    // For solveIK, we also use MOCK_METHOD.
    MOCK_METHOD(Result<AxisSet, IKError>, solveIK, (const CartPose& pose, const AxisSet& seed_joints), (const, override));

    MOCK_METHOD(void, setHomePosition, (const AxisSet& home_joints), (override));
    MOCK_METHOD(AxisSet, getHomePosition, (), (const, override));
};

} // namespace RDT

#endif // MOCK_KINEMATIC_SOLVER_H