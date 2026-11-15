// Responsibility: Verifies immutable correlation, ranking, scene evidence, and resolved-task rules.

#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "manipulation/workflow/transfer_workflow_ports.hpp"
#include "transfer_domain_test_support.hpp"

namespace manipulation::workflow
{
namespace
{

using test_support::grasp_candidate;
using test_support::observation;
using test_support::scene_access_proof;
using test_support::scene_snapshot;
using test_support::stamped_pose;

static_assert(std::is_same_v<
    decltype(std::declval<const PlanningSceneSnapshot &>().scene()),
    const moveit_msgs::msg::PlanningScene &>);
static_assert(std::is_same_v<
    decltype(std::declval<const ResolvedTransferTask &>().scene_access_proof()),
    const PlanningSceneAccessProof &>);

TEST(TransferWorkflowTypes, ObservationRetainsStampFrameAndCorrelation)
{
  const auto value = observation("camera-detection-12");

  EXPECT_EQ(value.object_id(), "cube_1");
  EXPECT_EQ(value.pose().header.frame_id, "world");
  EXPECT_EQ(value.correlation().frame_id(), "world");
  EXPECT_EQ(value.correlation().stamp().sec, 42);
  EXPECT_EQ(value.correlation().stamp().nanosec, 123U);
  EXPECT_EQ(value.correlation().correlation_id(), "camera-detection-12");
  ASSERT_TRUE(value.confidence().has_value());
  EXPECT_DOUBLE_EQ(*value.confidence(), 0.9);
}

TEST(TransferWorkflowTypes, RanksOnlyExactlyCorrelatedCandidatesStably)
{
  const auto observed = observation();
  std::vector<GraspCandidate> candidates;
  candidates.emplace_back(grasp_candidate(observed, 0.2, 0.02));
  candidates.emplace_back(grasp_candidate(observed, 0.9, 0.03));
  candidates.emplace_back(grasp_candidate(observed, 0.9, 0.04));

  const auto ranked = rank_grasp_candidates(observed, std::move(candidates));

  ASSERT_EQ(ranked.size(), 3U);
  EXPECT_DOUBLE_EQ(ranked.at(0).score(), 0.9);
  EXPECT_DOUBLE_EQ(ranked.at(0).gripper_opening_m(), 0.03);
  EXPECT_DOUBLE_EQ(ranked.at(1).gripper_opening_m(), 0.04);
  EXPECT_DOUBLE_EQ(ranked.at(2).score(), 0.2);
  EXPECT_EQ(ranked.front().correlation(), observed.correlation());

  const auto another_observation = ObjectObservation::create(
    "cube_1", stamped_pose(), "another-correlation");
  EXPECT_THROW(
    rank_grasp_candidates(
      observed, std::vector<GraspCandidate>{grasp_candidate(another_observation)}),
    std::invalid_argument);
}

TEST(TransferWorkflowTypes, SnapshotOwnsReadOnlyCompleteSceneAndRevision)
{
  moveit_msgs::msg::PlanningScene source;
  source.name = "revision-seven";
  source.is_diff = false;
  const auto snapshot = PlanningSceneSnapshot::create("coordinator-a", 7U, source);
  source.name = "mutated-source";

  EXPECT_EQ(snapshot.coordinator_session_id(), "coordinator-a");
  EXPECT_EQ(snapshot.revision(), 7U);
  EXPECT_EQ(snapshot.scene().name, "revision-seven");

  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  EXPECT_THROW(
    PlanningSceneSnapshot::create("coordinator-a", 8U, std::move(diff)),
    std::invalid_argument);

  moveit_msgs::msg::PlanningScene robot_state_diff;
  robot_state_diff.is_diff = false;
  robot_state_diff.robot_state.is_diff = true;
  EXPECT_THROW(
    PlanningSceneSnapshot::create(
      "coordinator-a", 9U, std::move(robot_state_diff)),
    std::invalid_argument);
}

TEST(TransferWorkflowTypes, ResolvedTaskRequiresMatchingSceneAccessProof)
{
  auto observed = observation();
  auto grasp = grasp_candidate(observed);
  const auto task = ResolvedTransferTask::create(
    test_support::instance(), test_support::profile(), observed, grasp,
    scene_snapshot(7U, "coordinator-a"), scene_access_proof(7U, "coordinator-a"),
    stamped_pose("world", 0.5), true, TransferRecoveryPolicy::kRetreatAndHome);

  EXPECT_EQ(task.scene_snapshot().revision(), 7U);
  EXPECT_EQ(task.scene_access_proof().revision(), 7U);
  EXPECT_EQ(task.scene_access_proof().coordinator_session_id(), "coordinator-a");
  EXPECT_EQ(task.scene_access_proof().lease_id(), "lease-1");

  EXPECT_THROW(
    ResolvedTransferTask::create(
      test_support::instance(), test_support::profile(), observed, grasp,
      scene_snapshot(7U, "coordinator-a"), scene_access_proof(8U, "coordinator-a"),
      stamped_pose(), false, TransferRecoveryPolicy::kNone),
    std::invalid_argument);
  EXPECT_THROW(
    ResolvedTransferTask::create(
      test_support::instance(), test_support::profile(), observed, grasp,
      scene_snapshot(7U, "coordinator-a"), scene_access_proof(7U, "coordinator-b"),
      stamped_pose(), false, TransferRecoveryPolicy::kNone),
    std::invalid_argument);
}

class RevisionRecordingGraspProvider final : public GraspProvider
{
public:
  GraspProviderResult generate(
    const ObjectObservation & observed,
    const ObjectProfile &,
    const PlanningSceneSnapshot & snapshot,
    TransferExecutionContext &) override
  {
    seen_revision = snapshot.revision();
    seen_scene_name = snapshot.scene().name;
    return GraspProviderResult::success({grasp_candidate(observed)});
  }

  void request_stop() noexcept override
  {
    stopped = true;
  }

  std::uint64_t seen_revision{0U};
  std::string seen_scene_name;
  bool stopped{false};
};

TEST(TransferWorkflowTypes, ProviderPortReceivesExplicitReadOnlySceneEvidence)
{
  static_assert(std::is_abstract_v<ObservationProvider>);
  static_assert(std::is_abstract_v<GraspProvider>);
  static_assert(std::is_abstract_v<TransferExecutionStrategy>);
}

}  // namespace
}  // namespace manipulation::workflow
