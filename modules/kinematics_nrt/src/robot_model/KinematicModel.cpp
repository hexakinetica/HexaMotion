// KinematicModel.cpp
#include "KinematicModel.h"
#include "LoggingMacros.h"
#include "DataTypes.h"
#include <kdl/chain.hpp>
// #include <kdl_parser/kdl_parser.hpp> // Disabled: kdl_parser missing

#include <cmath> 

namespace RDT {

// Use constants from Units.h since M_PI is not standard
constexpr double M_PI_VAL = UnitConstants::PI;
constexpr double M_PI_2_VAL = UnitConstants::PI / 2.0;

// PIMPL implementation struct
struct KinematicModel::Impl {
    KDL::Chain chain;
};

// --- Lifecycle ---
KinematicModel::KinematicModel() : pimpl_(std::make_unique<Impl>()) {}
KinematicModel::~KinematicModel() = default;
KinematicModel::KinematicModel(KinematicModel&&) noexcept = default;
KinematicModel& KinematicModel::operator=(KinematicModel&&) noexcept = default;


std::unique_ptr<KinematicModel> KinematicModel::createFromURDF(const std::string& urdf_string,
                                                               const std::string& base_link,
                                                               const std::string& tip_link) {
    // MOCK IMPLEMENTATION: Returns hardcoded model as fallback
    RDT_LOG_WARN(MODULE_NAME, "URDF Parsing is disabled (kdl_parser missing). Returning hardcoded fallback model.");
    return createLbrIisy11R1300();
    
    /* Original implementation:
    auto model = std::unique_ptr<KinematicModel>(new KinematicModel());
    KDL::Tree tree;

    if (!kdl_parser::treeFromString(urdf_string, tree)) {
        RDT_LOG_ERROR(MODULE_NAME, "Failed to parse URDF string.");
        return nullptr;
    }

    if (!tree.getChain(base_link, tip_link, model->pimpl_->chain)) {
        RDT_LOG_ERROR(MODULE_NAME, "Failed to extract chain from '{}' to '{}' from URDF tree.", base_link, tip_link);
        return nullptr;
    }

    if (model->getNrOfJoints() != ROBOT_AXES_COUNT) {
        RDT_LOG_ERROR(MODULE_NAME, "Extracted chain has {} joints, but {} are required.", model->getNrOfJoints(), ROBOT_AXES_COUNT);
        return nullptr;
    }
    
    RDT_LOG_INFO(MODULE_NAME, "KinematicModel created successfully from URDF ({} -> {}), {} DoF.", base_link, tip_link, model->getNrOfJoints());
    return model;
    */
}

std::unique_ptr<KinematicModel> KinematicModel::createLbrIisy11R1300() {
    RDT_LOG_INFO(MODULE_NAME, "Creating hardcoded KinematicModel for KUKA LBR iisy 11 R1300.");
    // Use new because constructor is private and make_unique can't see it
    auto model = std::unique_ptr<KinematicModel>(new KinematicModel());
    KDL::Chain& chain = model->pimpl_->chain;

    // --- Geometry Definition (Derived from URDF) ---
    // URDF structure: Parent Link -> Joint Origin (Transform) -> Joint Axis -> Child Link
    // KDL structure: Segment = Joint (at base of segment) -> Frame (transform to tip of segment)

    chain.addSegment(KDL::Segment("base_link_to_joint1", KDL::Joint(KDL::Joint::None),
        KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.315))));
    chain.addSegment(KDL::Segment("link1", KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Rotation::RPY(M_PI_2_VAL, 0.0, M_PI_VAL), KDL::Vector(0.0, 0.0, 0.085))));
    chain.addSegment(KDL::Segment("link2", KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Rotation::RPY(-M_PI_2_VAL, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0))));
    chain.addSegment(KDL::Segment("link3", KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Rotation::RPY(-M_PI_2_VAL, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.085))));
    chain.addSegment(KDL::Segment("link4", KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Rotation::RPY(M_PI_2_VAL, 0.0, 0.0), KDL::Vector(0.0, 0.4, 0.0))));
    chain.addSegment(KDL::Segment("link5", KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Rotation::RPY(M_PI_2_VAL, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.085))));
    chain.addSegment(KDL::Segment("link6_to_flange", KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, M_PI_VAL), KDL::Vector(0.0, 0.0, 0.0))));


    if (model->getNrOfJoints() != ROBOT_AXES_COUNT) {
        RDT_LOG_CRITICAL(MODULE_NAME, "Hardcoded chain DOF ({}) does not match ROBOT_AXES_COUNT ({}).", model->getNrOfJoints(), ROBOT_AXES_COUNT);
        return nullptr;
    }

    RDT_LOG_INFO(MODULE_NAME, "KUKA LBR iisy 11 R1300 hardcoded model created with {} DoF.", model->getNrOfJoints());
    return model;
}

const KDL::Chain& KinematicModel::getChain() const {
    return pimpl_->chain;
}

unsigned int KinematicModel::getNrOfJoints() const {
    return pimpl_->chain.getNrOfJoints();
}

} // namespace RDT
