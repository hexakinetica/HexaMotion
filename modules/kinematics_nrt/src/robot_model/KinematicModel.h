// KinematicModel.h
#ifndef KINEMATIC_MODEL_H
#define KINEMATIC_MODEL_H

#pragma once

#include <string>
#include <memory>

// Forward-declare KDL types to hide them from the public interface (PIMPL)
namespace KDL {
    class Chain;
}

namespace RDT {

/**
 * @class KinematicModel
 * @brief Represents the geometric structure of a robot manipulator.
 *
 * This class is responsible for loading and providing the robot's kinematic chain,
 * typically from a URDF description. It strictly deals with the robot's geometry
 * (links and joints) and does not contain information about physical limits like
 * velocity or position constraints.
 *
 * This class uses the PIMPL idiom to hide the underlying KDL dependency from its public header.
 * @version 2.0 (Refactored)
 */
class KinematicModel {
public:
    /**
     * @brief Destructor.
     */
    ~KinematicModel();

    // The model owns a unique resource (pimpl_), so it's non-copyable but movable.
    KinematicModel(const KinematicModel&) = delete;
    KinematicModel& operator=(const KinematicModel&) = delete;
    KinematicModel(KinematicModel&&) noexcept;
    KinematicModel& operator=(KinematicModel&&) noexcept;

    /**
     * @brief Factory method to create a KinematicModel from a URDF file content.
     * @param urdf_string A string containing the entire content of a URDF file.
     * @param base_link The name of the root link in the chain (e.g., "base_link").
     * @param tip_link The name of the final link in the chain (e.g., "flange").
     * @return A unique_ptr to the created KinematicModel on success, or nullptr on failure.
     */
    [[nodiscard]] static std::unique_ptr<KinematicModel> createFromURDF(const std::string& urdf_string,
                                                                        const std::string& base_link,
                                                                        const std::string& tip_link);

    /**
     * @brief [DEPRECATED] Factory method to create a hardcoded KinematicModel for the KUKA LBR iisy.
     * This method is for testing and will be removed once URDF loading is standard.
     */
    [[nodiscard]] static std::unique_ptr<KinematicModel> createLbrIisy11R1300();

    /**
     * @brief Gets a const reference to the underlying KDL kinematic chain.
     * This is intended for use by internal components like KdlKinematicSolver.
     * @return A const reference to the KDL::Chain.
     */
    [[nodiscard]] const KDL::Chain& getChain() const;

    /**
     * @brief Gets the number of joints in the kinematic chain.
     * @return The number of degrees of freedom.
     */
    [[nodiscard]] unsigned int getNrOfJoints() const;

private:
    // Private constructor to be used by factory methods.
    KinematicModel();

    // PIMPL: Pointer to implementation
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    
    static inline const std::string MODULE_NAME = "KinematicModel";
};

} // namespace RDT
#endif // KINEMATIC_MODEL_H
