// Responsibility: Verifies config-driven perception readiness and fail-closed degradation policy.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "manipulation/monitoring/health_aggregator.hpp"
#include "manipulation/monitoring/perception_health_policy.hpp"

namespace
{

using namespace std::chrono_literals;
using manipulation::monitoring::classifyPerceptionPipeline;
using manipulation::monitoring::ComponentHealth;
using manipulation::monitoring::findPerceptionPipeline;
using manipulation::monitoring::FreshnessStatus;
using manipulation::monitoring::HealthAggregator;
using manipulation::monitoring::HealthComponentId;
using manipulation::monitoring::HealthInputs;
using manipulation::monitoring::HealthIssueCode;
using manipulation::monitoring::HealthLevel;
using manipulation::monitoring::makePerceptionPipelineDescriptors;
using manipulation::monitoring::PerceptionPipelineDescriptor;
using manipulation::monitoring::PerceptionPipelineInput;
using manipulation::monitoring::validatePerceptionDiagnosticBindings;

constexpr char kDetectionTask[] = "perception/object_detection";

bool hasIssue(const ComponentHealth & component, HealthIssueCode issue)
{
  return std::find(component.issues.begin(), component.issues.end(), issue) !=
         component.issues.end();
}

HealthInputs healthyInputs()
{
  HealthInputs inputs;
  inputs.joint_states = FreshnessStatus{HealthLevel::kOk, true, false, 10ms};
  inputs.joint_profile_configured = true;
  inputs.joint_stamp_ns = 1;
  inputs.controllers = FreshnessStatus{HealthLevel::kOk, true, false, 10ms};
  inputs.controller_profile_configured = true;
  inputs.robot_tf_available = true;
  inputs.motion_backend_available = true;
  inputs.gripper_available = true;
  inputs.planning_scene_service_available = true;
  inputs.planning_scene_state = FreshnessStatus{HealthLevel::kOk, true, false, 10ms};
  inputs.planning_scene_backend_ready = true;
  inputs.planning_scene_operation_stuck_after = 30s;
  return inputs;
}

PerceptionPipelineInput detectionInput(bool required)
{
  return PerceptionPipelineInput{
    HealthComponentId::kObjectDetection,
    kDetectionTask,
    required,
    FreshnessStatus{HealthLevel::kOk, true, false, 20ms},
    HealthLevel::kOk,
    "pipeline observed"};
}

TEST(PerceptionHealthPolicy, BuildsDescriptorsFromExplicitComponentIds)
{
  const auto descriptors = makePerceptionPipelineDescriptors(
    {"custom/depth", "custom/detection", "custom/pose"},
    {
      static_cast<std::int64_t>(HealthComponentId::kStereoPerception),
      static_cast<std::int64_t>(HealthComponentId::kObjectDetection),
      static_cast<std::int64_t>(HealthComponentId::kPoseEstimation)
    });

  ASSERT_EQ(descriptors.size(), 3U);
  EXPECT_EQ(descriptors[0].component_id, HealthComponentId::kStereoPerception);
  EXPECT_EQ(descriptors[1].component_id, HealthComponentId::kObjectDetection);
  EXPECT_EQ(descriptors[2].component_id, HealthComponentId::kPoseEstimation);
}

TEST(PerceptionHealthPolicy, ResolvesOnlyExactConfiguredTaskNames)
{
  const std::vector<PerceptionPipelineDescriptor> descriptors{
    {kDetectionTask, HealthComponentId::kObjectDetection},
    {"perception/object_pose", HealthComponentId::kPoseEstimation}};

  const auto known = findPerceptionPipeline(descriptors, kDetectionTask);
  ASSERT_TRUE(known.has_value());
  EXPECT_EQ(known->component_id, HealthComponentId::kObjectDetection);
  EXPECT_FALSE(findPerceptionPipeline(descriptors, "node: perception/object_detection"));
  EXPECT_FALSE(findPerceptionPipeline(descriptors, "object_detection"));
}

TEST(PerceptionHealthPolicy, AllowsArbitraryBackendNeutralTaskNames)
{
  const std::vector<PerceptionPipelineDescriptor> descriptors{
    {"vendor_x/custom_detector", HealthComponentId::kObjectDetection}};

  EXPECT_NO_THROW(validatePerceptionDiagnosticBindings(
      descriptors, {"runtime_node: vendor_x/custom_detector"},
      {"vendor_x/custom_detector"}));
}

TEST(PerceptionHealthPolicy, AllowsAlternativeProducersForOneStableTask)
{
  const std::vector<PerceptionPipelineDescriptor> descriptors{
    {"perception/depth", HealthComponentId::kStereoPerception},
    {"perception/depth", HealthComponentId::kStereoPerception}};

  EXPECT_NO_THROW(validatePerceptionDiagnosticBindings(
      descriptors,
      {"primary_depth: perception/depth", "fallback_depth: perception/depth"},
      {"perception/depth"}));
}

TEST(PerceptionHealthPolicy, RejectsMalformedDescriptorParameters)
{
  EXPECT_THROW(
    makePerceptionPipelineDescriptors(
      {"perception/depth"}, {}),
    std::invalid_argument);
  EXPECT_THROW(
    makePerceptionPipelineDescriptors(
      {""},
      {static_cast<std::int64_t>(HealthComponentId::kStereoPerception)}),
    std::invalid_argument);
  EXPECT_THROW(
    makePerceptionPipelineDescriptors(
      {"perception/depth"},
      {static_cast<std::int64_t>(HealthComponentId::kMotionBackend)}),
    std::invalid_argument);
}

TEST(PerceptionHealthPolicy, RejectsIncompleteOrAmbiguousBindings)
{
  const std::vector<PerceptionPipelineDescriptor> detection{
    {kDetectionTask, HealthComponentId::kObjectDetection}};
  EXPECT_THROW(
    validatePerceptionDiagnosticBindings(detection, {}, {}),
    std::invalid_argument);
  EXPECT_THROW(
    validatePerceptionDiagnosticBindings(detection, {""}, {}),
    std::invalid_argument);
  EXPECT_THROW(
    validatePerceptionDiagnosticBindings({}, {}, {kDetectionTask}),
    std::invalid_argument);
  EXPECT_THROW(
    validatePerceptionDiagnosticBindings(
      detection, {"detector: perception/object_detection"},
      {kDetectionTask, kDetectionTask}),
    std::invalid_argument);

  const std::vector<PerceptionPipelineDescriptor> conflicting{
    {"perception/result", HealthComponentId::kObjectDetection},
    {"perception/result", HealthComponentId::kPoseEstimation}};
  EXPECT_THROW(
    validatePerceptionDiagnosticBindings(
      conflicting, {"detector result", "pose result"}, {}),
    std::invalid_argument);
}

TEST(PerceptionHealthPolicy, RejectsDuplicateWireNamesAndExactPairs)
{
  const std::vector<PerceptionPipelineDescriptor> distinct{
    {"perception/depth", HealthComponentId::kStereoPerception},
    {kDetectionTask, HealthComponentId::kObjectDetection}};
  EXPECT_THROW(
    validatePerceptionDiagnosticBindings(
      distinct, {"duplicate wire status", "duplicate wire status"}, {}),
    std::invalid_argument);

  const std::vector<PerceptionPipelineDescriptor> repeated{
    {kDetectionTask, HealthComponentId::kObjectDetection},
    {kDetectionTask, HealthComponentId::kObjectDetection}};
  EXPECT_THROW(
    validatePerceptionDiagnosticBindings(
      repeated, {"same detector", "same detector"}, {}),
    std::invalid_argument);
}

TEST(PerceptionHealthPolicy, RequiredUnseenPipelineFailsClosedDuringStartupGrace)
{
  auto input = detectionInput(true);
  input.freshness = FreshnessStatus{HealthLevel::kWarn, false, true, 100ms};

  const auto component = classifyPerceptionPipeline(input);

  EXPECT_TRUE(component.required);
  EXPECT_FALSE(component.observed);
  EXPECT_EQ(component.level, HealthLevel::kWarn);
  EXPECT_TRUE(component.startup_grace_active);
  EXPECT_TRUE(hasIssue(
      component, HealthIssueCode::kPerceptionDiagnosticUnavailable));

  auto inputs = healthyInputs();
  inputs.perception_pipelines.push_back(input);
  const auto health = HealthAggregator{}.aggregate(inputs);
  EXPECT_FALSE(health.motion_ready);
  EXPECT_EQ(health.overall_level, HealthLevel::kWarn);
}

TEST(PerceptionHealthPolicy, OptionalUnseenPipelineDoesNotBlockMotion)
{
  auto input = detectionInput(false);
  input.freshness = FreshnessStatus{HealthLevel::kStale, false, false, 6s};
  auto inputs = healthyInputs();
  inputs.perception_pipelines.push_back(input);

  const auto health = HealthAggregator{}.aggregate(inputs);

  EXPECT_TRUE(health.motion_ready);
  EXPECT_EQ(health.overall_level, HealthLevel::kOk);
  ASSERT_EQ(health.components.size(), 8U);
}

TEST(PerceptionHealthPolicy, MalformedUnseenOkInputStillFailsClosed)
{
  auto input = detectionInput(true);
  input.freshness = FreshnessStatus{HealthLevel::kOk, false, false, 6s};
  const auto component = classifyPerceptionPipeline(input);
  EXPECT_EQ(component.level, HealthLevel::kStale);
}

TEST(PerceptionHealthPolicy, FreshUpstreamWarnHasAStableTypedReason)
{
  auto input = detectionInput(true);
  input.reported_level = HealthLevel::kWarn;
  input.reported_detail = "processing p95 exceeded budget";

  const auto component = classifyPerceptionPipeline(input);

  EXPECT_EQ(component.level, HealthLevel::kWarn);
  EXPECT_EQ(component.detail, input.reported_detail);
  EXPECT_TRUE(hasIssue(
      component, HealthIssueCode::kPerceptionPipelineReportedWarn));
}

TEST(PerceptionHealthPolicy, ReceiptStalenessOverridesAnOldOkReport)
{
  auto input = detectionInput(true);
  input.freshness = FreshnessStatus{HealthLevel::kStale, true, false, 6s};

  const auto component = classifyPerceptionPipeline(input);

  EXPECT_EQ(component.level, HealthLevel::kStale);
  EXPECT_TRUE(component.observed);
  EXPECT_TRUE(hasIssue(
      component, HealthIssueCode::kPerceptionDiagnosticStale));
}

TEST(PerceptionHealthPolicy, FreshUpstreamErrorHasAStableTypedReason)
{
  auto input = detectionInput(true);
  input.reported_level = HealthLevel::kError;
  const auto component = classifyPerceptionPipeline(input);
  EXPECT_EQ(component.level, HealthLevel::kError);
  EXPECT_TRUE(hasIssue(
      component, HealthIssueCode::kPerceptionPipelineReportedError));
}

TEST(PerceptionHealthPolicy, UnknownDiagnosticLevelFailsClosed)
{
  EXPECT_EQ(
    manipulation::monitoring::healthLevelFromDiagnostic(255U),
    HealthLevel::kError);
}

TEST(PerceptionHealthPolicy, RejectsNonPerceptionComponentInput)
{
  auto input = detectionInput(true);
  input.id = HealthComponentId::kMotionBackend;
  EXPECT_THROW(classifyPerceptionPipeline(input), std::invalid_argument);
}

TEST(PerceptionHealthPolicy, RejectsEmptyTaskInput)
{
  auto input = detectionInput(true);
  input.task_name.clear();
  EXPECT_THROW(classifyPerceptionPipeline(input), std::invalid_argument);
}

TEST(PerceptionHealthPolicy, AggregateRejectsDuplicateTypedComponents)
{
  auto inputs = healthyInputs();
  inputs.perception_pipelines = {detectionInput(false), detectionInput(false)};
  EXPECT_THROW(HealthAggregator{}.aggregate(inputs), std::invalid_argument);
}

}  // namespace
