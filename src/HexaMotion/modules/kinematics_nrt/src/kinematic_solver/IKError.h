// IKError.h
#ifndef IK_ERROR_H
#define IK_ERROR_H

#pragma once

#include <string>
#include <cstdint>

namespace RDT {

/**
 * @enum IKError
 * @brief Defines specific error types for Inverse Kinematics (IK) failures.
 */
enum class IKError : uint8_t {
    Unreachable,    ///< The target pose is outside the robot's reachable workspace.
    Singularity,    ///< The robot is in or near a singular configuration.
    SolverTimeout,  ///< The iterative solver did not converge within the allowed time/iterations.
    InternalError   ///< An unexpected error occurred within the solver.
};

/**
 * @brief Converts an IKError enum value to a human-readable string.
 * @param error The error code to convert.
 * @return A string describing the error.
 */
[[nodiscard]] inline std::string ToString(IKError error) {
    switch (error) {
        case IKError::Unreachable:    return "Target pose is unreachable";
        case IKError::Singularity:    return "Robot is near a singularity";
        case IKError::SolverTimeout:  return "Solver failed to converge (timeout)";
        case IKError::InternalError:  return "An internal solver error occurred";
        default:                      return "Unknown IK Error";
    }
}

} // namespace RDT

#endif // IK_ERROR_H