// Responsibility: Verifies deterministic aggregation of subsystem health into system readiness.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>

#include "manipulation/monitoring/health_aggregator.hpp"

namespace
{

using namespace std::chrono_literals;
using manipulation::monitoring::ComponentHealth;
using manipulation::monitoring::findComponent;
using manipulation::monitoring::FreshnessStatus;
using manipulation::monitoring::HealthAggregator;
using manipulation::monitoring::HealthComponentId;
using manipulation::monitoring::HealthInputs;
using manipulation::monitoring::HealthIssueCode;
using manipulation::monitoring::HealthLevel;
using manipulation::monitoring::ObservationAgeSemantics;
using manipulation::monitoring::SystemHealth;

HealthInputs healthyInputs()
{
  HealthInputs inputs;
  inputs.sequence = 42U;
  inputs.steady_uptime = 5s;
  inputs.joint_states = FreshnessStatus{HealthLevel::kOk, true, false, 20ms};
  inputs.joint_profile_configured = true;
  inputs.joint_stamp_ns = 1;
  inputs.controllers = FreshnessStatus{HealthLevel::kOk, true, false, 30ms};
  inputs.controller_profile_configured = true;
  inputs.robot_tf_available = true;
  inputs.motion_backend_available = true;
  inputs.gripper_available = true;
  inputs.planning_scene_service_available = true;
  inputs.planning_scene_state = FreshnessStatus{HealthLevel::kOk, true, false, 40ms};
  inputs.planning_scene_backend_ready = true;
  inputs.planning_scene_operation_stuck_after = 30s;
  return inputs;
}

bool hasIssue(const ComponentHealth & component, HealthIssueCode issue)
{
  return std::find(component.issues.begin(), component.issues.end(), issue) !=
         component.issues.end();
}

TEST(HealthAggregator, ReportsReadyOnlyWhenEveryRequiredComponentIsOk)
{
  const auto health = HealthAggregator{}.aggregate(healthyInputs());

  EXPECT_TRUE(health.motion_ready);
  EXPECT_EQ(health.overall_level, HealthLevel::kOk);
  EXPECT_EQ(health.sequence, 42U);
  EXPECT_EQ(health.steady_uptime, 5s);
  ASSERT_EQ(health.components.size(), 7U);

  const auto & clock = findComponent(health, HealthComponentId::kSimulationClock);
  EXPECT_FALSE(clock.required);
  EXPECT_EQ(clock.level, HealthLevel::kOk);
  EXPECT_EQ(clock.age_semantics, ObservationAgeSemantics::kNotApplicable);
}

TEST(HealthAggregator, MotionBackendRenamePreservesStableNumericValues)
{
  EXPECT_EQ(static_cast<std::uint8_t>(HealthComponentId::kMotionBackend), 5U);
  EXPECT_EQ(
    static_cast<std::uint16_t>(HealthIssueCode::kMotionBackendUnavailable),
    500U);
}

TEST(HealthAggregator, StartupGraceIsTypedWarnAndBlocksMotion)
{
  auto inputs = healthyInputs();
  inputs.startup_grace_active = true;
  inputs.simulation_clock_required = true;
  inputs.joint_states = FreshnessStatus{HealthLevel::kWarn, false, true, 500ms};
  inputs.simulation_clock = FreshnessStatus{HealthLevel::kWarn, false, true, 500ms};
  inputs.controllers = FreshnessStatus{HealthLevel::kWarn, false, true, 500ms};
  inputs.robot_tf_available = false;
  inputs.motion_backend_available = false;
  inputs.gripper_available = false;
  inputs.planning_scene_service_available = false;
  inputs.planning_scene_state =
    FreshnessStatus{HealthLevel::kWarn, false, true, 500ms};

  const auto health = HealthAggregator{}.aggregate(inputs);

  EXPECT_FALSE(health.motion_ready);
  EXPECT_EQ(health.overall_level, HealthLevel::kWarn);
  const auto & joints = findComponent(health, HealthComponentId::kJointStates);
  EXPECT_TRUE(joints.startup_grace_active);
  EXPECT_FALSE(joints.observed);
  EXPECT_EQ(joints.age_semantics, ObservationAgeSemantics::kSinceMonitorStart);
  EXPECT_EQ(joints.observation_age, 500ms);
  EXPECT_TRUE(hasIssue(joints, HealthIssueCode::kJointStatesUnavailable));
  EXPECT_TRUE(hasIssue(
      findComponent(health, HealthComponentId::kSimulationClock),
      HealthIssueCode::kSimulationClockUnavailable));
  EXPECT_TRUE(hasIssue(
      findComponent(health, HealthComponentId::kRobotTf),
      HealthIssueCode::kRobotTfUnavailable));
  EXPECT_TRUE(hasIssue(
      findComponent(health, HealthComponentId::kPlanningScene),
      HealthIssueCode::kPlanningSceneCoordinatorUnavailable));
}

TEST(HealthAggregator, StaleStreamsAndUnavailableBackendsHaveStableCodes)
{
  auto inputs = healthyInputs();
  inputs.joint_states = FreshnessStatus{HealthLevel::kStale, true, false, 3s};
  inputs.controllers = FreshnessStatus{HealthLevel::kStale, true, false, 6s};
  inputs.robot_tf_available = false;
  inputs.motion_backend_available = false;
  inputs.gripper_available = false;

  const auto health = HealthAggregator{}.aggregate(inputs);

  EXPECT_FALSE(health.motion_ready);
  EXPECT_EQ(health.overall_level, HealthLevel::kStale);
  const auto & joints = findComponent(health, HealthComponentId::kJointStates);
  EXPECT_TRUE(joints.observed);
  EXPECT_EQ(joints.age_semantics, ObservationAgeSemantics::kSinceLastObservation);
  EXPECT_TRUE(hasIssue(joints, HealthIssueCode::kJointStatesStale));
  EXPECT_TRUE(hasIssue(
      findComponent(health, HealthComponentId::kControllers),
      HealthIssueCode::kControllerStateStale));
  EXPECT_EQ(
    findComponent(health, HealthComponentId::kMotionBackend).level,
    HealthLevel::kError);
  EXPECT_TRUE(hasIssue(
      findComponent(health, HealthComponentId::kGripper),
      HealthIssueCode::kGripperUnavailable));
}

TEST(HealthAggregator, ContentFailuresElevateFreshObservationsToError)
{
  auto inputs = healthyInputs();
  inputs.missing_joints = {"arm_joint_7"};
  inputs.joint_stamp_ns = 0;
  inputs.inactive_controllers = {"arm_controller"};

  const auto health = HealthAggregator{}.aggregate(inputs);

  EXPECT_FALSE(health.motion_ready);
  EXPECT_EQ(health.overall_level, HealthLevel::kError);
  const auto & joints = findComponent(health, HealthComponentId::kJointStates);
  EXPECT_EQ(joints.level, HealthLevel::kError);
  EXPECT_TRUE(hasIssue(joints, HealthIssueCode::kRequiredJointsMissing));
  EXPECT_TRUE(hasIssue(joints, HealthIssueCode::kJointStampMissing));
  const auto & controllers = findComponent(health, HealthComponentId::kControllers);
  EXPECT_EQ(controllers.level, HealthLevel::kError);
  EXPECT_TRUE(hasIssue(
      controllers, HealthIssueCode::kRequiredControllersInactive));
}

TEST(HealthAggregator, MissingRobotProfileFailsClosed)
{
  auto inputs = healthyInputs();
  inputs.joint_profile_configured = false;
  inputs.controller_profile_configured = false;

  const auto health = HealthAggregator{}.aggregate(inputs);

  EXPECT_FALSE(health.motion_ready);
  EXPECT_EQ(health.overall_level, HealthLevel::kError);
  EXPECT_TRUE(hasIssue(
      findComponent(health, HealthComponentId::kJointStates),
      HealthIssueCode::kRequiredJointsMissing));
  EXPECT_TRUE(hasIssue(
      findComponent(health, HealthComponentId::kControllers),
      HealthIssueCode::kRequiredControllersInactive));
}

TEST(HealthAggregator, DelayedWarnIsNotAdvertisedAsMotionReady)
{
  auto inputs = healthyInputs();
  inputs.joint_states = FreshnessStatus{HealthLevel::kWarn, true, false, 750ms};

  const auto health = HealthAggregator{}.aggregate(inputs);

  EXPECT_FALSE(health.motion_ready);
  EXPECT_EQ(health.overall_level, HealthLevel::kWarn);
  EXPECT_TRUE(hasIssue(
      findComponent(health, HealthComponentId::kJointStates),
      HealthIssueCode::kJointStatesDelayed));
}

TEST(HealthAggregator, JointStampToleranceHasASeparateReasonCode)
{
  auto inputs = healthyInputs();
  inputs.joint_stamp_age_sec = -3.0;
  inputs.joint_stamp_outside_tolerance = true;

  const auto health = HealthAggregator{}.aggregate(inputs);
  const auto & joints = findComponent(health, HealthComponentId::kJointStates);

  EXPECT_EQ(joints.level, HealthLevel::kError);
  EXPECT_TRUE(hasIssue(
      joints, HealthIssueCode::kJointStampOutsideTolerance));
}

TEST(HealthAggregator, BriefPlanningSceneOperationIsWarnButNotError)
{
  auto inputs = healthyInputs();
  inputs.planning_scene_operation_in_progress = true;
  inputs.planning_scene_operation_age = 2s;

  const auto health = HealthAggregator{}.aggregate(inputs);
  const auto & scene = findComponent(
    health, HealthComponentId::kPlanningScene);

  EXPECT_FALSE(health.motion_ready);
  EXPECT_EQ(scene.level, HealthLevel::kWarn);
  EXPECT_TRUE(hasIssue(
      scene, HealthIssueCode::kPlanningSceneOperationInProgress));
  EXPECT_FALSE(hasIssue(
      scene, HealthIssueCode::kPlanningSceneOperationStuck));
}

TEST(HealthAggregator, PlanningSceneFreshnessUsesDistinctTypedReasons)
{
  auto delayed_inputs = healthyInputs();
  delayed_inputs.planning_scene_state =
    FreshnessStatus{HealthLevel::kWarn, true, false, 3s};
  const auto delayed = HealthAggregator{}.aggregate(delayed_inputs);
  EXPECT_TRUE(hasIssue(
      findComponent(delayed, HealthComponentId::kPlanningScene),
      HealthIssueCode::kPlanningSceneStateDelayed));

  auto stale_inputs = healthyInputs();
  stale_inputs.planning_scene_state =
    FreshnessStatus{HealthLevel::kStale, true, false, 6s};
  const auto stale = HealthAggregator{}.aggregate(stale_inputs);
  const auto & scene = findComponent(stale, HealthComponentId::kPlanningScene);
  EXPECT_EQ(scene.level, HealthLevel::kStale);
  EXPECT_TRUE(hasIssue(
      scene, HealthIssueCode::kPlanningSceneStateStale));
}

TEST(HealthAggregator, PlanningSceneServiceLossIsImmediateAfterStartup)
{
  auto inputs = healthyInputs();
  inputs.planning_scene_service_available = false;

  const auto health = HealthAggregator{}.aggregate(inputs);
  const auto & scene = findComponent(
    health, HealthComponentId::kPlanningScene);

  EXPECT_EQ(scene.level, HealthLevel::kError);
  EXPECT_TRUE(hasIssue(
      scene, HealthIssueCode::kPlanningSceneCoordinatorUnavailable));
}

TEST(HealthAggregator, PlanningSceneCoordinatorConsistencyFaultsAreTypedAndFailClosed)
{
  auto inputs = healthyInputs();
  inputs.planning_scene_backend_ready = false;
  inputs.planning_scene_backend_error_code = 107U;
  inputs.planning_scene_reconciliation_required = true;
  inputs.planning_scene_unreconciled_lease_expiry_count = 2U;
  inputs.planning_scene_operation_capacity_exhausted = true;

  const auto health = HealthAggregator{}.aggregate(inputs);
  const auto & scene = findComponent(health, HealthComponentId::kPlanningScene);

  EXPECT_FALSE(health.motion_ready);
  EXPECT_EQ(scene.level, HealthLevel::kError);
  EXPECT_TRUE(hasIssue(scene, HealthIssueCode::kPlanningSceneBackendUnavailable));
  EXPECT_TRUE(hasIssue(scene, HealthIssueCode::kPlanningSceneBackendError));
  EXPECT_TRUE(hasIssue(scene, HealthIssueCode::kPlanningSceneReconciliationRequired));
  EXPECT_TRUE(hasIssue(scene, HealthIssueCode::kPlanningSceneLeaseExpired));
  EXPECT_TRUE(hasIssue(
      scene, HealthIssueCode::kPlanningSceneOperationCapacityExhausted));
}

TEST(HealthAggregator, PlanningSceneOperationBecomesErrorAfterSteadyLimit)
{
  auto inputs = healthyInputs();
  inputs.planning_scene_operation_in_progress = true;
  inputs.planning_scene_operation_age = 30s;

  const auto health = HealthAggregator{}.aggregate(inputs);
  const auto & scene = findComponent(
    health, HealthComponentId::kPlanningScene);

  EXPECT_EQ(scene.level, HealthLevel::kError);
  EXPECT_TRUE(hasIssue(
      scene, HealthIssueCode::kPlanningSceneOperationStuck));
}

TEST(HealthAggregator, PlanningSceneOperationRequiresAPositiveLimit)
{
  auto inputs = healthyInputs();
  inputs.planning_scene_operation_in_progress = true;
  inputs.planning_scene_operation_stuck_after = 0s;

  EXPECT_THROW(HealthAggregator{}.aggregate(inputs), std::invalid_argument);
}

TEST(HealthAggregator, MissingComponentLookupFailsFast)
{
  EXPECT_THROW(
    findComponent(SystemHealth{}, HealthComponentId::kJointStates),
    std::logic_error);
}

}  // namespace
