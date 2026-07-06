#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "planner/TrajectoryPlanner.h"
#include "MotionManager.h" 
#include "HardwareManager.h"
#include "RobotConfig.h" // For ControllerConfig
#include <thread>
#include <chrono>

using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;

namespace {
class LocalMockKinematicSolver : public KinematicSolver {
public:
    MOCK_METHOD(bool, solveFK, (const AxisSet& joints, CartPose& result), (const, override));
    MOCK_METHOD((Result<AxisSet, IKError>), solveIK, (const CartPose& pose, const AxisSet& seed_joints), (const, override));
    MOCK_METHOD(void, setHomePosition, (const AxisSet& home_joints), (override));
    MOCK_METHOD(AxisSet, getHomePosition, (), (const, override));
};
}

// A test fixture to set up the full stack: Planner -> MotionManager -> HardwareManager (sim)
class TrajectoryPlannerIntegrationTest : public ::testing::Test {
protected:
    InterfaceConfig hw_config;
    RobotLimits limits;
    ControllerConfig ctrl_config;

    void SetUp() override {
        // Kinematics
        solver_ = std::make_shared<LocalMockKinematicSolver>();

        // Setup default mock behaviors
        ON_CALL(*solver_, solveIK)
            .WillByDefault([](const CartPose& pose, const AxisSet& seed) {
                (void)pose;
                return Result<AxisSet, IKError>::Success(seed);
            });
        ON_CALL(*solver_, solveFK)
            .WillByDefault([](const AxisSet& joints, CartPose& result) {
                (void)joints;
                result = CartPose{};
                return true;
            });

        // HAL
        limits.joint_position_limits_deg.fill({-180.0_deg, 180.0_deg});
        limits.joint_velocity_limits_deg_s.fill(1000.0_deg_s);
        
        // Sim driver setup
        hw_config.realtime_type = InterfaceConfig::RealtimeInterfaceType::None;
        auto hw_manager = std::make_shared<HardwareManager>(hw_config, limits);
        ASSERT_TRUE(hw_manager->init().isSuccess());

        // Motion Manager
        motion_manager = std::make_shared<MotionManager>(hw_manager, 40, limits, 20.0_deg);
        ASSERT_TRUE(motion_manager->start());

        // Planner Config
        ctrl_config.PlannerTickSec = 0.004_s; // 250 Hz execution
        // Note: Preview uses hardcoded ~30Hz internally in generatePreviewPath

        // Planner
        auto interpolator = std::make_shared<TrajectoryInterpolator>(solver_);
        planner = std::make_unique<TrajectoryPlanner>(interpolator, motion_manager, ctrl_config);

        // Set initial state. The origin pose is declared VALID explicitly: validity is a flag, not
        // a value comparison, and Cartesian segments refuse to plan from an invalid start pose.
        TrajectoryPoint initial_state;
        initial_state.command.cartesian_valid = true;
        planner->setCurrentState(initial_state);
    }

    void TearDown() override {
        motion_manager->stop();
    }

    std::shared_ptr<LocalMockKinematicSolver> solver_;
    std::shared_ptr<MotionManager> motion_manager;
    std::unique_ptr<TrajectoryPlanner> planner;

    // Runs the NRT/RT loop until the planner reports the task finished or a safety cap is reached.
    // Returns the last feedback sample seen.
    TrajectoryPoint runUntilFinished(int max_iterations = 2000) {
        TrajectoryPoint last_fb;
        int loop_count = 0;
        while (!planner->isTaskFinished() && loop_count < max_iterations) {
            planner->update();
            TrajectoryPoint fb;
            while (motion_manager->dequeueFeedback(fb)) {
                last_fb = fb;
            }
            std::this_thread::sleep_for(5ms);
            loop_count++;
        }
        return last_fb;
    }
};

TEST_F(TrajectoryPlannerIntegrationTest, AddSingleTargetAndExecute) {
    TrajectoryPoint target;
    target.header.motion_type = MotionType::JOINT;
    target.command.joint_target.SetFromPositionArray({10.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    target.command.speed_ratio = 0.1;

    auto plan_res = planner->addTargetWaypoint(target);
    ASSERT_TRUE(plan_res.isSuccess());

    int loop_count = 0;
    while (!planner->isTaskFinished() && loop_count < 500) {
        planner->update();
        TrajectoryPoint fb;
        while(motion_manager->dequeueFeedback(fb)) {}
        std::this_thread::sleep_for(20ms);
        loop_count++;
    }
    
    ASSERT_TRUE(planner->isTaskFinished());
}

TEST_F(TrajectoryPlannerIntegrationTest, PreviewGenerationIsSparse) {
    // 1. Create a program with one movement long enough to render many execution ticks.
    NetProtocol::ProgramDataStruct prog;
    prog.name = "PreviewTest";

    NetProtocol::ProgramStepStruct step;
    step.id = 1;
    step.type = NetProtocol::StepType::MoveJ;
    // Move A1 from 0 to 20 deg.
    // DEFAULT_JOINT_V_MAX is 360 deg/s (TrajectoryInterpolator.h). Speed ratio 0.1 -> 36 deg/s,
    // a = 72 deg/s^2 -> trapezoid with 0.5 s ramps, ~1.06 s total.
    step.joint_target.SetFromPositionArray({20.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    step.speed_ratio = 10.0; // 10%
    prog.steps.push_back(step);

    // 2. Generate preview
    auto path = planner->generatePreviewPath(prog);

    // 3. Verify the preview is DOWNSAMPLED (60 Hz preview_dt), not execution-dense (250 Hz):
    // ~1.06 s * 60 Hz ~= 64 preview points vs ~264 execution ticks for the same move.
    size_t point_count = path.points.size();

    std::cout << "[TEST] Generated " << point_count << " preview points." << std::endl;

    EXPECT_GT(point_count, 30);   // the move is actually rendered, not collapsed to endpoints
    EXPECT_LT(point_count, 130);  // and clearly below the 250 Hz execution density (~264)

    // Waypoints should be captured
    EXPECT_EQ(path.waypoints.size(), 1);
}

TEST_F(TrajectoryPlannerIntegrationTest, OverrideTrajectoryClearsQueue) {
    TrajectoryPoint target;
    target.header.motion_type = MotionType::JOINT;
    target.command.joint_target.SetFromPositionArray({5.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    target.command.speed_ratio = 0.2;

    ASSERT_TRUE(planner->addTargetWaypoint(target).isSuccess());

    planner->update();
    EXPECT_GT(motion_manager->getCommandQueueSize(), 0u);

    TrajectoryPoint override_target = target;
    override_target.command.joint_target.SetFromPositionArray({-5.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    auto override_res = planner->overrideTrajectory(override_target);
    ASSERT_TRUE(override_res.isSuccess());

    EXPECT_EQ(motion_manager->getCommandQueueSize(), 0u);
    planner->update();
    EXPECT_GT(motion_manager->getCommandQueueSize(), 0u);
}

TEST_F(TrajectoryPlannerIntegrationTest, OverrideTrajectoryPlansHoldForNonPlannableFeedbackTypes) {
    // STOP/PAUSE hold-replan regression (REQ-PLAN-06): RobotController::stopProgram()/pauseProgram()
    // pass the latest FEEDBACK point to overrideTrajectory(). Feedback echoes what the RT loop was
    // executing: MotionType::HOLD when the RT buffer is empty (robot already standing), SPLINE or
    // CIRC when stopped mid-move — none of which is plannable as a single segment by createSegment()
    // (HOLD/SPLINE are not single-segment types; a CIRC with coincident start/via/target has no arc
    // geometry). The override must plan the hold as a zero-length JOINT segment and succeed for
    // every feedback type instead of failing with UnsupportedMotionType (planner error 3).
    for (const MotionType feedback_type : {MotionType::HOLD, MotionType::CIRC, MotionType::SPLINE}) {
        TrajectoryPoint feedback;
        feedback.header.motion_type = feedback_type;
        feedback.command.joint_target.SetFromPositionArray(
            {5.0_deg, -10.0_deg, 15.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});

        auto res = planner->overrideTrajectory(feedback);
        ASSERT_TRUE(res.isSuccess())
            << "hold replan failed for feedback motion type " << static_cast<int>(feedback_type)
            << " with planner error " << static_cast<int>(res.error());

        // A hold segment must actually have been planned (trajectory not empty before update()).
        EXPECT_FALSE(planner->isTaskFinished());
    }
}

TEST_F(TrajectoryPlannerIntegrationTest, UpdateDoesNotOverfillBuffer) {
    TrajectoryPoint target;
    target.header.motion_type = MotionType::JOINT;
    target.command.joint_target.SetFromPositionArray({20.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    target.command.speed_ratio = 0.1;

    ASSERT_TRUE(planner->addTargetWaypoint(target).isSuccess());

    planner->update();
    EXPECT_LE(motion_manager->getCommandQueueSize(), 50u);
}

TEST_F(TrajectoryPlannerIntegrationTest, InterpolatorPopulatesTargetJointPose) {
    auto solver = std::make_shared<LocalMockKinematicSolver>();
    TrajectoryInterpolator interpolator(solver);

    TrajectoryPoint start;
    start.command.joint_target.SetFromPositionArray({0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    
    TrajectoryPoint target;
    target.header.motion_type = MotionType::JOINT;
    target.command.joint_target.SetFromPositionArray({15.0_deg, -5.0_deg, 10.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    target.command.speed_ratio = 1.0;

    auto res = interpolator.createSegment(start, target, 0.01_s);
    ASSERT_TRUE(res.isSuccess());

    auto segment = std::move(res.value());
    const auto& points = segment->getPoints();
    ASSERT_FALSE(points.empty());

    auto expected_target = target.command.joint_target.ToPositionArray();
    
    // Total motion duration should be the actual physical time to complete the segment,
    // not just the 0.01s 'dt' simulation step.
    double calculated_duration = points.front().segment_target.motion_duration.value();
    EXPECT_GT(calculated_duration, 0.01); // Needs to be greater than just 1 dt tick

    for (const auto& pt : points) {
        EXPECT_EQ(pt.segment_target.target_angles[0].value(), expected_target[0].value());
        EXPECT_EQ(pt.segment_target.target_angles[1].value(), expected_target[1].value());
        EXPECT_EQ(pt.segment_target.target_angles[2].value(), expected_target[2].value());
        // Or simply compare the array itself if Degrees supports operator==
        EXPECT_EQ(pt.segment_target.target_angles, expected_target);
        EXPECT_DOUBLE_EQ(pt.segment_target.motion_duration.value(), calculated_duration);
    }
}

TEST_F(TrajectoryPlannerIntegrationTest, PlanMotionChainEmptyIsSuccessNoQueue) {
    // An empty chain must be a safe no-op: success, nothing queued, task considered finished.
    auto res = planner->planMotionChain({});
    ASSERT_TRUE(res.isSuccess());
    planner->update();
    EXPECT_EQ(motion_manager->getCommandQueueSize(), 0u);
    EXPECT_TRUE(planner->isTaskFinished());
}

TEST_F(TrajectoryPlannerIntegrationTest, PlanMotionChainFinePointsExecuteToFinalTarget) {
    // Three fine (radius 0) JOINT waypoints planned as one continuous chain must execute to completion
    // and land exactly on the final target. This protects the look-ahead production path (REQ-PLAN-09).
    std::vector<TrajectoryPoint> chain;
    const std::array<std::array<Degrees, 6>, 3> targets = {{
        {{10.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg}},
        {{10.0_deg, 10.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg}},
        {{20.0_deg, 10.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg}},
    }};
    for (uint32_t i = 0; i < targets.size(); ++i) {
        TrajectoryPoint tp;
        tp.header.motion_type = MotionType::JOINT;
        tp.header.sequence_index = i;
        tp.header.blending_radius = 0.0_mm; // fine points
        tp.command.joint_target.SetFromPositionArray(targets[i]);
        tp.command.speed_ratio = 0.5;
        chain.push_back(tp);
    }

    ASSERT_TRUE(planner->planMotionChain(chain).isSuccess());

    TrajectoryPoint last_fb = runUntilFinished();
    ASSERT_TRUE(planner->isTaskFinished());

    auto final_joints = last_fb.feedback.joint_actual.ToPositionArray();
    EXPECT_NEAR(final_joints[0].value(), 20.0, 1.0);
    EXPECT_NEAR(final_joints[1].value(), 10.0, 1.0);
}

TEST_F(TrajectoryPlannerIntegrationTest, PlanMotionChainWithBlendExecutesToFinalTarget) {
    // A blended (radius > 0) corner must not deadlock the velocity guard and must still finish exactly
    // on the FINAL waypoint (the corner is approximated, the end is exact) — REQ-PLAN-10.
    // Map FK joints -> Cartesian (A1 deg -> x mm, A2 deg -> y mm) so the blend window K is non-zero.
    ON_CALL(*solver_, solveFK)
        .WillByDefault([](const AxisSet& joints, CartPose& result) {
            result = CartPose{};
            result.x = Millimeters(joints[AxisId::A1].position.value());
            result.y = Millimeters(joints[AxisId::A2].position.value());
            return true;
        });

    // L-shaped path: start (0,0) -> wp0 (A1=30) -> wp1 (A1=30, A2=30). Blend the wp0 corner.
    std::vector<TrajectoryPoint> chain;
    TrajectoryPoint wp0;
    wp0.header.motion_type = MotionType::JOINT;
    wp0.header.sequence_index = 0;
    wp0.header.blending_radius = 8.0_mm; // round this corner
    wp0.command.joint_target.SetFromPositionArray({30.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    wp0.command.speed_ratio = 0.5;
    chain.push_back(wp0);

    TrajectoryPoint wp1;
    wp1.header.motion_type = MotionType::JOINT;
    wp1.header.sequence_index = 1;
    wp1.header.blending_radius = 0.0_mm; // final point is exact
    wp1.command.joint_target.SetFromPositionArray({30.0_deg, 30.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    wp1.command.speed_ratio = 0.5;
    chain.push_back(wp1);

    ASSERT_TRUE(planner->planMotionChain(chain).isSuccess());

    TrajectoryPoint last_fb = runUntilFinished();
    ASSERT_TRUE(planner->isTaskFinished());

    auto final_joints = last_fb.feedback.joint_actual.ToPositionArray();
    EXPECT_NEAR(final_joints[0].value(), 30.0, 1.0);
    EXPECT_NEAR(final_joints[1].value(), 30.0, 1.0);
}

// ---------------------------------------------------------------------------
// CIRC (CircMotionProfile) — docs/REQ_motion_circ.md
// ---------------------------------------------------------------------------
namespace {
CartPose makeCartPose(double x_mm, double y_mm, double z_mm,
                      double rx_deg = 0.0, double ry_deg = 0.0, double rz_deg = 0.0) {
    CartPose pose{};
    pose.x = Millimeters(x_mm);
    pose.y = Millimeters(y_mm);
    pose.z = Millimeters(z_mm);
    pose.rx = Degrees(rx_deg);
    pose.ry = Degrees(ry_deg);
    pose.rz = Degrees(rz_deg);
    return pose;
}

double distanceMm(const CartPose& p, double x_mm, double y_mm, double z_mm) {
    const double dx = p.x.value() - x_mm;
    const double dy = p.y.value() - y_mm;
    const double dz = p.z.value() - z_mm;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
} // namespace

TEST(CircMotionProfileTest, HalfCircleStaysOnCircleAndEndsExactly) {
    // Half circle in the XY plane: center (0,0,0), radius 100 mm, CCW through the +Y via point.
    const CartPose start = makeCartPose(100.0, 0.0, 0.0);
    const CartPose via = makeCartPose(0.0, 100.0, 0.0);
    const CartPose end = makeCartPose(-100.0, 0.0, 0.0);

    CircMotionProfile profile(start, via, end, 250.0_mm_s, 500.0_mm_s2);
    ASSERT_TRUE(profile.isValid());
    ASSERT_GT(profile.getDuration().value(), 0.0);
    EXPECT_EQ(profile.getMotionType(), MotionType::CIRC);

    // Every sample must lie on the circle (radius 100 from the origin) and in the plane z = 0.
    const Seconds duration = profile.getDuration();
    double min_dist_to_via = 1.0e9;
    const int kSamples = 200;
    for (int i = 0; i <= kSamples; ++i) {
        const Seconds t = Seconds(duration.value() * static_cast<double>(i) / kSamples);
        auto res = profile.interpolateCartesian(t);
        ASSERT_TRUE(res.isSuccess());
        const CartPose p = res.value();
        EXPECT_NEAR(distanceMm(p, 0.0, 0.0, 0.0), 100.0, 1.0e-6);
        EXPECT_NEAR(p.z.value(), 0.0, 1.0e-6);
        min_dist_to_via = std::min(min_dist_to_via, distanceMm(p, 0.0, 100.0, 0.0));
    }
    // The traversed arc must actually visit the via point (direction correctness, REQ-CIRC-01).
    EXPECT_LT(min_dist_to_via, 2.0);

    // Endpoint is exact.
    auto end_res = profile.interpolateCartesian(duration);
    ASSERT_TRUE(end_res.isSuccess());
    EXPECT_LT(distanceMm(end_res.value(), -100.0, 0.0, 0.0), 1.0e-6);

    // Arc length = pi * R, so duration must exceed the straight-line (diameter) LIN duration.
    LinMotionProfile lin(start, end, 250.0_mm_s, 500.0_mm_s2);
    EXPECT_GT(profile.getDuration().value(), lin.getDuration().value());
}

TEST(CircMotionProfileTest, DirectionFollowsViaClockwise) {
    // Same circle, but the via at -Y forces the CLOCKWISE traversal (270 degrees of sweep) even
    // though the CCW arc to the end point would be shorter (90 degrees).
    const CartPose start = makeCartPose(100.0, 0.0, 0.0);
    const CartPose via = makeCartPose(0.0, -100.0, 0.0);
    const CartPose end = makeCartPose(0.0, 100.0, 0.0);

    CircMotionProfile profile(start, via, end, 250.0_mm_s, 500.0_mm_s2);
    ASSERT_TRUE(profile.isValid());

    const Seconds duration = profile.getDuration();
    double min_dist_to_via = 1.0e9;
    const int kSamples = 300;
    for (int i = 0; i <= kSamples; ++i) {
        const Seconds t = Seconds(duration.value() * static_cast<double>(i) / kSamples);
        auto res = profile.interpolateCartesian(t);
        ASSERT_TRUE(res.isSuccess());
        min_dist_to_via = std::min(min_dist_to_via, distanceMm(res.value(), 0.0, -100.0, 0.0));
    }
    EXPECT_LT(min_dist_to_via, 2.0);

    // 270-degree sweep is 3x the arc of the 90-degree shortcut: the CCW quarter-arc profile through
    // the +X..+Y quadrant must be strictly shorter in time.
    CircMotionProfile quarter(start, makeCartPose(std::sqrt(0.5) * 100.0, std::sqrt(0.5) * 100.0, 0.0),
                              end, 250.0_mm_s, 500.0_mm_s2);
    ASSERT_TRUE(quarter.isValid());
    EXPECT_GT(profile.getDuration().value(), quarter.getDuration().value());
}

TEST(CircMotionProfileTest, OrientationSlerpsFromStartToTarget) {
    // Quarter circle with a 90-degree TCP rotation about Z. The trapezoid is symmetric, so at
    // t = T/2 half the arc is travelled and the slerp must be half-way (rz ~ 45 deg).
    const CartPose start = makeCartPose(100.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const CartPose via = makeCartPose(std::sqrt(0.5) * 100.0, std::sqrt(0.5) * 100.0, 0.0);
    const CartPose end = makeCartPose(0.0, 100.0, 0.0, 0.0, 0.0, 90.0);

    CircMotionProfile profile(start, via, end, 250.0_mm_s, 500.0_mm_s2);
    ASSERT_TRUE(profile.isValid());

    auto at_start = profile.interpolateCartesian(0.0_s);
    auto at_mid = profile.interpolateCartesian(Seconds(profile.getDuration().value() / 2.0));
    auto at_end = profile.interpolateCartesian(profile.getDuration());
    ASSERT_TRUE(at_start.isSuccess());
    ASSERT_TRUE(at_mid.isSuccess());
    ASSERT_TRUE(at_end.isSuccess());
    EXPECT_NEAR(at_start.value().rz.value(), 0.0, 1.0e-6);
    EXPECT_NEAR(at_mid.value().rz.value(), 45.0, 1.0);
    EXPECT_NEAR(at_end.value().rz.value(), 90.0, 1.0e-6);
}

TEST(CircMotionProfileTest, RejectsDegenerateGeometryWithTypedErrors) {
    const CartPose p0 = makeCartPose(0.0, 0.0, 0.0);
    const CartPose p1 = makeCartPose(50.0, 0.0, 0.0);
    const CartPose p2 = makeCartPose(100.0, 0.0, 0.0);

    // Collinear points: no unique circle plane.
    CircMotionProfile collinear(p0, p1, p2, 250.0_mm_s, 500.0_mm_s2);
    EXPECT_FALSE(collinear.isValid());
    EXPECT_EQ(collinear.geometryError(), CircMotionProfile::GeometryError::CollinearPoints);

    // Coincident via/start.
    CircMotionProfile coincident(p0, p0, p2, 250.0_mm_s, 500.0_mm_s2);
    EXPECT_FALSE(coincident.isValid());
    EXPECT_EQ(coincident.geometryError(), CircMotionProfile::GeometryError::CoincidentPoints);

    // Coincident via/end.
    CircMotionProfile coincident_end(p0, p2, p2, 250.0_mm_s, 500.0_mm_s2);
    EXPECT_FALSE(coincident_end.isValid());
    EXPECT_EQ(coincident_end.geometryError(), CircMotionProfile::GeometryError::CoincidentPoints);

    // An invalid profile must return typed errors from interpolation, never garbage poses.
    auto res = collinear.interpolateCartesian(0.1_s);
    EXPECT_TRUE(res.isError());
}

TEST_F(TrajectoryPlannerIntegrationTest, InterpolatorCreatesCircSegment) {
    TrajectoryInterpolator interpolator(solver_);

    TrajectoryPoint start;
    start.command.cartesian_target = makeCartPose(100.0, 0.0, 0.0);
    start.command.cartesian_valid = true;

    TrajectoryPoint target;
    target.header.motion_type = MotionType::CIRC;
    target.command.cartesian_via = makeCartPose(0.0, 100.0, 0.0);
    target.command.cartesian_target = makeCartPose(-100.0, 0.0, 0.0);
    target.command.speed_ratio = 1.0;

    auto res = interpolator.createSegment(start, target, 0.01_s);
    ASSERT_TRUE(res.isSuccess());

    const auto& points = res.value()->getPoints();
    ASSERT_GT(points.size(), 10u); // half circle, R=100mm: a real dense rendering, not a stub
    for (const auto& pt : points) {
        // Every rendered point (except copies of the authored target) lies on the circle.
        EXPECT_NEAR(distanceMm(pt.command.cartesian_target, 0.0, 0.0, 0.0), 100.0, 1.0e-3);
    }
    EXPECT_TRUE(points.back().header.is_target_reached_for_this_point);
    EXPECT_LT(distanceMm(points.back().command.cartesian_target, -100.0, 0.0, 0.0), 1.0e-6);
}

TEST_F(TrajectoryPlannerIntegrationTest, InterpolatorRejectsInvalidCircGeometry) {
    TrajectoryInterpolator interpolator(solver_);

    TrajectoryPoint start;
    start.command.cartesian_target = makeCartPose(0.0, 0.0, 0.0);
    start.command.cartesian_valid = true;

    TrajectoryPoint target;
    target.header.motion_type = MotionType::CIRC;
    target.command.cartesian_via = makeCartPose(50.0, 0.0, 0.0);      // collinear with start/target
    target.command.cartesian_target = makeCartPose(100.0, 0.0, 0.0);
    target.command.speed_ratio = 1.0;

    auto res = interpolator.createSegment(start, target, 0.01_s);
    ASSERT_TRUE(res.isError());
    EXPECT_EQ(res.error(), TrajectoryInterpolator::PlannerError::InvalidPathGeometry);
}

TEST_F(TrajectoryPlannerIntegrationTest, PlanMotionChainWithCircExecutesToCompletion) {
    // A CIRC waypoint must flow through planMotionChain -> MotionManager exactly like LIN
    // (REQ-CIRC-05). The mock IK returns the seed, so this validates pipeline acceptance and
    // completion, not kinematics.
    std::vector<TrajectoryPoint> chain;
    TrajectoryPoint wp;
    wp.header.motion_type = MotionType::CIRC;
    wp.header.sequence_index = 0;
    wp.header.blending_radius = 0.0_mm;
    wp.command.cartesian_via = makeCartPose(50.0, 50.0, 0.0);
    wp.command.cartesian_target = makeCartPose(100.0, 0.0, 0.0);
    wp.command.cartesian_valid = true; // mirrors the RobotController wire boundary (Cartesian type)
    wp.command.speed_ratio = 1.0;
    chain.push_back(wp);

    ASSERT_TRUE(planner->planMotionChain(chain).isSuccess());
    (void)runUntilFinished();
    ASSERT_TRUE(planner->isTaskFinished());
}

// ---------------------------------------------------------------------------
// SPLINE (SplinePath / SplineMotionProfile) — docs/REQ_motion_spline.md
// ---------------------------------------------------------------------------

TEST(SplinePathTest, PassesThroughEveryPointAndIsMonotone) {
    const std::vector<Eigen::Vector3d> pts = {
        {0.0, 0.0, 0.0}, {100.0, 50.0, 0.0}, {200.0, -30.0, 20.0}, {300.0, 0.0, 0.0}};
    SplinePath path(pts);
    ASSERT_TRUE(path.isValid());
    ASSERT_EQ(path.spanCount(), 3);
    ASSERT_GT(path.totalLength(), 0.0);

    // The curve passes exactly through every input point at the span-boundary path lengths.
    double s_at_point[4] = {0.0, 0.0, 0.0, 0.0};
    s_at_point[3] = path.totalLength();
    // Recover interior boundary lengths from spanLocationAt by scanning: fraction wraps to 0/1.
    for (int span = 0; span < 3; ++span) {
        // Sample the end of each span: binary-search s where the span index switches.
        double lo = 0.0, hi = path.totalLength();
        for (int iter = 0; iter < 60; ++iter) {
            const double mid = 0.5 * (lo + hi);
            if (path.spanLocationAt(mid).span <= span) { lo = mid; } else { hi = mid; }
        }
        s_at_point[span + 1] = lo;
    }
    for (int i = 0; i < 4; ++i) {
        const Eigen::Vector3d p = path.positionAt(s_at_point[i]);
        EXPECT_LT((p - pts[static_cast<size_t>(i)]).norm(), 1.0e-3)
            << "point " << i << " missed by " << (p - pts[static_cast<size_t>(i)]).norm() << " mm";
    }

    // Arc-length parametrization is monotone: position advances with s, never retreats.
    Eigen::Vector3d prev = path.positionAt(0.0);
    double walked = 0.0;
    const int kSamples = 400;
    for (int k = 1; k <= kSamples; ++k) {
        const double s = path.totalLength() * k / kSamples;
        const Eigen::Vector3d p = path.positionAt(s);
        walked += (p - prev).norm();
        prev = p;
    }
    // Walked chord length matches the reported total length within the table resolution.
    EXPECT_NEAR(walked, path.totalLength(), path.totalLength() * 0.01);
}

TEST(SplinePathTest, TwoPointsIsExactStraightLine) {
    // REQ-SPL-09: the interpolating spline through two points is the straight segment.
    SplinePath path({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}});
    ASSERT_TRUE(path.isValid());
    EXPECT_NEAR(path.totalLength(), 100.0, 1.0e-6);
    for (int k = 0; k <= 10; ++k) {
        const Eigen::Vector3d p = path.positionAt(10.0 * k);
        EXPECT_NEAR(p.y(), 0.0, 1.0e-9);
        EXPECT_NEAR(p.z(), 0.0, 1.0e-9);
        EXPECT_NEAR(p.x(), 10.0 * k, 1.0e-3);
    }
    EXPECT_GE(path.minCurvatureRadius(), SplinePath::kStraightCurvatureRadius);
}

TEST(SplinePathTest, C1ContinuityAcrossSpanJoints) {
    const std::vector<Eigen::Vector3d> pts = {
        {0.0, 0.0, 0.0}, {100.0, 80.0, 0.0}, {200.0, 0.0, 0.0}};
    SplinePath path(pts);
    ASSERT_TRUE(path.isValid());
    // Numeric tangent immediately before and after the interior point must agree in direction
    // (C1 continuity of the centripetal Catmull-Rom).
    double lo = 0.0, hi = path.totalLength();
    for (int iter = 0; iter < 60; ++iter) {
        const double mid = 0.5 * (lo + hi);
        if (path.spanLocationAt(mid).span == 0) { lo = mid; } else { hi = mid; }
    }
    const double s_joint = lo;
    const double h = 0.5; // mm
    const Eigen::Vector3d tang_in = (path.positionAt(s_joint) - path.positionAt(s_joint - h)).normalized();
    const Eigen::Vector3d tang_out = (path.positionAt(s_joint + h) - path.positionAt(s_joint)).normalized();
    EXPECT_GT(tang_in.dot(tang_out), 0.999) << "tangent kink at the span joint";
}

TEST(SplinePathTest, RejectsDegenerateInput) {
    SplinePath too_few({{1.0, 2.0, 3.0}});
    EXPECT_FALSE(too_few.isValid());
    EXPECT_EQ(too_few.geometryError(), SplinePath::GeometryError::TooFewPoints);

    SplinePath coincident({{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}, {100.0, 0.000001, 0.0}});
    EXPECT_FALSE(coincident.isValid());
    EXPECT_EQ(coincident.geometryError(), SplinePath::GeometryError::CoincidentPoints);
}

TEST(SplineMotionProfileTest, CurvatureSpeedPlanSlowsCornerNotStraights) {
    // REQ-SPL-06 rev 2 (pointwise limiting): a hairpin corner must be traversed at the local
    // curvature speed limit, while the straight approach still reaches a real fraction of the
    // commanded speed. Rev 1's single global cap drove the WHOLE block (including the 100 mm
    // straight approach) at the corner speed — the "robot slows down towards the end of the
    // trajectory" defect on long blocks.
    const std::vector<CartPose> hairpin = {
        makeCartPose(100.0, 0.0, 0.0), makeCartPose(110.0, 10.0, 0.0), makeCartPose(100.0, 20.0, 0.0)};
    constexpr double kVMax = 1000.0;   // mm/s (profile v_max below; a_max = 2000 mm/s^2)
    SplineMotionProfile tight(makeCartPose(0.0, 0.0, 0.0), hairpin, 1000.0_mm_s, 2000.0_mm_s2);
    ASSERT_TRUE(tight.isValid());

    // Sample the commanded speed by central differences over the rendered Cartesian positions.
    const double duration_s = tight.getDuration().value();
    ASSERT_GT(duration_s, 0.0);
    const double dt = 0.002;
    const Eigen::Vector3d corner(110.0, 10.0, 0.0);
    double v_peak_overall = 0.0;
    double v_peak_near_corner = 0.0;
    for (double t = dt; t + dt < duration_s; t += dt) {
        auto before = tight.interpolateCartesian(Seconds(t - dt));
        auto here = tight.interpolateCartesian(Seconds(t));
        auto after = tight.interpolateCartesian(Seconds(t + dt));
        ASSERT_TRUE(before.isSuccess());
        ASSERT_TRUE(here.isSuccess());
        ASSERT_TRUE(after.isSuccess());
        const auto pos = [](const CartPose& p) {
            return Eigen::Vector3d(p.x.value(), p.y.value(), p.z.value());
        };
        const double v = (pos(after.value()) - pos(before.value())).norm() / (2.0 * dt);
        v_peak_overall = std::max(v_peak_overall, v);
        if ((pos(here.value()) - corner).norm() <= 10.0) {
            v_peak_near_corner = std::max(v_peak_near_corner, v);
        }
    }

    // Corner: within 10 mm of the apex the speed must respect the local curvature limit.
    // The apex radius of this hairpin is ~5-15 mm -> sqrt(a_max * R) ~ 100-170 mm/s; allow the
    // plan's sampling discretization some headroom but stay an order below v_max.
    EXPECT_LT(v_peak_near_corner, 300.0);
    // Straight approach: the block must actually speed up away from the corner. The 100 mm
    // approach limited by braking into the corner supports > 400 mm/s; rev 1's global cap held
    // this below ~170 mm/s.
    EXPECT_GT(v_peak_overall, 400.0);
    EXPECT_LE(v_peak_overall, kVMax * 1.05);
}

TEST(SplineMotionProfileTest, OrientationSlerpsPerSpan) {
    // Orientation must land exactly on each authored orientation at its point (REQ-SPL-07).
    const std::vector<CartPose> block = {
        makeCartPose(100.0, 0.0, 0.0, 0.0, 0.0, 30.0),
        makeCartPose(200.0, 50.0, 0.0, 0.0, 0.0, 60.0)};
    SplineMotionProfile profile(makeCartPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), block,
                                250.0_mm_s, 500.0_mm_s2);
    ASSERT_TRUE(profile.isValid());
    auto at_start = profile.interpolateCartesian(0.0_s);
    auto at_end = profile.interpolateCartesian(profile.getDuration());
    ASSERT_TRUE(at_start.isSuccess());
    ASSERT_TRUE(at_end.isSuccess());
    EXPECT_NEAR(at_start.value().rz.value(), 0.0, 1.0e-6);
    EXPECT_NEAR(at_end.value().rz.value(), 60.0, 1.0e-6);
    // End position is the last authored point, exactly.
    EXPECT_LT(distanceMm(at_end.value(), 200.0, 50.0, 0.0), 1.0e-3);
}

TEST(SplineMotionProfileTest, LeadingDuplicateHandling) {
    // Start == first point, same orientation: legal, the duplicate is dropped.
    const CartPose start = makeCartPose(100.0, 0.0, 0.0);
    const std::vector<CartPose> block = {makeCartPose(100.0, 0.0, 0.0), makeCartPose(200.0, 0.0, 0.0)};
    SplineMotionProfile ok(start, block, 250.0_mm_s, 500.0_mm_s2);
    ASSERT_TRUE(ok.isValid());
    EXPECT_GT(ok.getDuration().value(), 0.0);

    // Start == first point but orientation differs: pure reorientation is LIN's job -> typed error.
    const std::vector<CartPose> reorient = {makeCartPose(100.0, 0.0, 0.0, 0.0, 0.0, 45.0),
                                            makeCartPose(200.0, 0.0, 0.0)};
    SplineMotionProfile bad(start, reorient, 250.0_mm_s, 500.0_mm_s2);
    EXPECT_FALSE(bad.isValid());
    EXPECT_EQ(bad.geometryError(), SplineMotionProfile::GeometryError::StartCoincidesWithFirstPoint);
}

TEST_F(TrajectoryPlannerIntegrationTest, InterpolatorCreatesSplineSegment) {
    TrajectoryInterpolator interpolator(solver_);

    TrajectoryPoint start;
    start.command.cartesian_target = makeCartPose(0.0, 0.0, 0.0);
    start.command.cartesian_valid = true;

    std::vector<TrajectoryPoint> block;
    const std::array<std::array<double, 3>, 3> pts = {{{100.0, 50.0, 0.0}, {200.0, -30.0, 0.0}, {300.0, 0.0, 0.0}}};
    for (uint32_t k = 0; k < pts.size(); ++k) {
        TrajectoryPoint wp;
        wp.header.motion_type = MotionType::SPLINE;
        wp.header.sequence_index = 10 + k;
        wp.command.cartesian_target = makeCartPose(pts[k][0], pts[k][1], pts[k][2]);
        wp.command.speed_ratio = 1.0;
        block.push_back(wp);
    }

    auto res = interpolator.createSplineSegment(start, block, 0.01_s);
    ASSERT_TRUE(res.isSuccess());
    const auto& points = res.value()->getPoints();
    ASSERT_GT(points.size(), 10u);

    // No stop flag inside the block; only the exit is an exact stop (REQ-SPL-02).
    for (size_t k = 0; k + 1 < points.size(); ++k) {
        EXPECT_FALSE(points[k].header.is_target_reached_for_this_point);
    }
    EXPECT_TRUE(points.back().header.is_target_reached_for_this_point);
    // Endpoint exact; sequence_index advances through the block for the HMI line display.
    EXPECT_LT(distanceMm(points.back().command.cartesian_target, 300.0, 0.0, 0.0), 1.0e-3);
    EXPECT_EQ(points.front().header.sequence_index, 10u);
    EXPECT_EQ(points.back().header.sequence_index, 12u);
}

TEST_F(TrajectoryPlannerIntegrationTest, InterpolatorRejectsCoincidentSplinePoints) {
    TrajectoryInterpolator interpolator(solver_);
    TrajectoryPoint start;
    start.command.cartesian_target = makeCartPose(0.0, 0.0, 0.0);
    start.command.cartesian_valid = true;

    std::vector<TrajectoryPoint> block;
    for (int k = 0; k < 2; ++k) {
        TrajectoryPoint wp;
        wp.header.motion_type = MotionType::SPLINE;
        wp.command.cartesian_target = makeCartPose(100.0, 0.0, 0.0); // both points identical
        wp.command.speed_ratio = 1.0;
        block.push_back(wp);
    }
    auto res = interpolator.createSplineSegment(start, block, 0.01_s);
    ASSERT_TRUE(res.isError());
    EXPECT_EQ(res.error(), TrajectoryInterpolator::PlannerError::InvalidPathGeometry);
}

TEST_F(TrajectoryPlannerIntegrationTest, SplineJointVelocityGuardFires) {
    // An IK that amplifies Cartesian motion into huge joint jumps must be rejected with the typed
    // ExcessiveJointVelocity error (REQ-SPL-08), not streamed to the robot.
    ON_CALL(*solver_, solveIK)
        .WillByDefault([](const CartPose& pose, const AxisSet& seed) {
            (void)seed;
            AxisSet out;
            // 1 mm of X -> 5 deg of A1: a 300 mm spline at full speed demands far more than
            // DEFAULT_JOINT_V_MAX * dt per cycle.
            out[AxisId::A1].position = Degrees(pose.x.value() * 5.0);
            return Result<AxisSet, IKError>::Success(out);
        });

    TrajectoryInterpolator interpolator(solver_);
    TrajectoryPoint start;
    start.command.cartesian_target = makeCartPose(0.0, 0.0, 0.0);
    start.command.cartesian_valid = true;

    std::vector<TrajectoryPoint> block;
    TrajectoryPoint wp;
    wp.header.motion_type = MotionType::SPLINE;
    wp.command.cartesian_target = makeCartPose(300.0, 0.0, 0.0);
    wp.command.speed_ratio = 1.0;
    block.push_back(wp);

    auto res = interpolator.createSplineSegment(start, block, 0.01_s);
    ASSERT_TRUE(res.isError());
    EXPECT_EQ(res.error(), TrajectoryInterpolator::PlannerError::ExcessiveJointVelocity);
}

TEST_F(TrajectoryPlannerIntegrationTest, PreviewRendersCircAndSplineSteps) {
    // The 3D preview must draw MoveC arcs (batch-1b leftover fixed in 2b) and MoveS blocks —
    // previously both were silently skipped and the operator saw a gap in the path.
    NetProtocol::ProgramDataStruct prog;
    prog.name = "CircSplinePreview";

    NetProtocol::ProgramStepStruct lin;
    lin.id = 0;
    lin.type = NetProtocol::StepType::MoveL;
    lin.cartesian_target = CartPose{100.0_mm, 0.0_mm, 0.0_mm, 0.0_deg, 0.0_deg, 0.0_deg};
    lin.speed_ratio = 100.0;
    prog.steps.push_back(lin);

    NetProtocol::ProgramStepStruct circ;
    circ.id = 1;
    circ.type = NetProtocol::StepType::MoveC;
    circ.cartesian_via = CartPose{150.0_mm, 50.0_mm, 0.0_mm, 0.0_deg, 0.0_deg, 0.0_deg};
    circ.cartesian_target = CartPose{200.0_mm, 0.0_mm, 0.0_mm, 0.0_deg, 0.0_deg, 0.0_deg};
    circ.speed_ratio = 100.0;
    prog.steps.push_back(circ);

    for (uint32_t k = 0; k < 2; ++k) {
        NetProtocol::ProgramStepStruct spl;
        spl.id = 2 + k;
        spl.type = NetProtocol::StepType::MoveS;
        spl.cartesian_target = CartPose{Millimeters(250.0 + 50.0 * k), Millimeters(k == 0 ? 40.0 : 0.0),
                                        0.0_mm, 0.0_deg, 0.0_deg, 0.0_deg};
        spl.speed_ratio = 100.0;
        prog.steps.push_back(spl);
    }

    auto path = planner->generatePreviewPath(prog);
    // Dense preview points exist for all three motion kinds (LIN + CIRC arc + spline block).
    EXPECT_GT(path.points.size(), 30u);
    // Waypoint markers: LIN end + CIRC end + spline inner point + spline block end.
    EXPECT_EQ(path.waypoints.size(), 4u);
}

TEST_F(TrajectoryPlannerIntegrationTest, PlanMotionChainMixedWithSplineBlockExecutes) {
    // LIN + a 2-point spline block + PTP in one chain must plan and execute to completion
    // (REQ-SPL-01/10): the block renders as ONE segment, boundaries are exact stops.
    std::vector<TrajectoryPoint> chain;

    TrajectoryPoint lin;
    lin.header.motion_type = MotionType::LIN;
    lin.header.sequence_index = 0;
    lin.command.cartesian_target = makeCartPose(50.0, 0.0, 0.0);
    lin.command.cartesian_valid = true; // mirrors the RobotController wire boundary (Cartesian type)
    lin.command.speed_ratio = 1.0;
    chain.push_back(lin);

    for (uint32_t k = 0; k < 2; ++k) {
        TrajectoryPoint wp;
        wp.header.motion_type = MotionType::SPLINE;
        wp.header.sequence_index = 1 + k;
        wp.command.cartesian_target = makeCartPose(100.0 + 50.0 * k, 30.0 * (k % 2 == 0 ? 1.0 : -1.0), 0.0);
        wp.command.cartesian_valid = true;
        wp.command.speed_ratio = 1.0;
        chain.push_back(wp);
    }

    TrajectoryPoint ptp;
    ptp.header.motion_type = MotionType::JOINT;
    ptp.header.sequence_index = 3;
    ptp.command.joint_target.SetFromPositionArray({5.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg});
    ptp.command.speed_ratio = 0.5;
    chain.push_back(ptp);

    ASSERT_TRUE(planner->planMotionChain(chain).isSuccess());
    (void)runUntilFinished();
    ASSERT_TRUE(planner->isTaskFinished());
}

int main(int argc, char **argv) {
    RDT::Logger::Init({std::make_shared<RDT::ConsoleSink>()}, RDT::LogLevel::Info);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    RDT::Logger::Shutdown();
    return result;
}