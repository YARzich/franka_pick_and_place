// Responsibility: Declares the private MTC implementation of the generic V2 grasp port.

#ifndef WORKFLOW_V2__MTC_GRASP_PROVIDER_HPP_
#define WORKFLOW_V2__MTC_GRASP_PROVIDER_HPP_

#include <Eigen/Geometry>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit/robot_model/robot_model.hpp"
#include "moveit/task_constructor/task.h"
#include "std_msgs/msg/header.hpp"

#include "manipulation/workflow/transfer_workflow_ports.hpp"

namespace franka_motion::workflow_v2
{

/** Sampling/ranking data owned by one named grasp-provider profile. */
struct MtcGraspProfileConfig
{
  std::string arm_group_name;
  std::string eef_name;
  std::string ik_frame_link;
  std::string pregrasp_pose;
  std::string grasp_pose;
  Eigen::Isometry3d ik_frame_transform{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d rotation_axis{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d approach_direction{0.0, 0.0, -1.0};
  double angle_delta{0.0};
  double min_solution_distance{0.0};
  std::size_t max_ik_solutions{0U};
  std::size_t max_solutions{0U};
  double pregrasp_distance_m{0.0};
  double gripper_opening_m{0.0};
  double gripper_force_n{0.0};
};

using MtcGraspProfiles = std::map<std::string, MtcGraspProfileConfig>;

namespace detail
{

/** Express a model-frame link pose in the caller's workflow frame. */
[[nodiscard]] geometry_msgs::msg::PoseStamped express_link_pose_in_frame(
  const Eigen::Isometry3d & model_to_output_frame,
  const Eigen::Isometry3d & model_to_link_frame,
  const Eigen::Isometry3d & link_to_pose,
  std_msgs::msg::Header output_header);

/** Injectable operations around the blocking part of one production MTC task. */
struct FixedStateMtcTaskOperations
{
  std::function<void(moveit::task_constructor::Task &)> initialize;
  std::function<bool(moveit::task_constructor::Task &, std::size_t)> plan;
  std::function<void(moveit::task_constructor::Task &)> preempt;
  std::function<std::vector<manipulation::workflow::GraspCandidate>(
      moveit::task_constructor::Task &,
      const manipulation::workflow::ObjectObservation &,
      const MtcGraspProfileConfig &,
      const std::string &)> extract_candidates;
};

/** Construct the real Task::init/plan/preempt/solution operations. */
[[nodiscard]] FixedStateMtcTaskOperations make_default_mtc_task_operations();

}  // namespace detail

/** Injectable planning seam; tests do not need robot plugins or a ROS graph. */
class MtcSnapshotPlanner
{
public:
  virtual ~MtcSnapshotPlanner() = default;

  [[nodiscard]] virtual manipulation::workflow::GraspProviderResult plan(
    const manipulation::workflow::ObjectObservation & observation,
    const manipulation::workflow::ObjectProfile & profile,
    const manipulation::workflow::PlanningSceneSnapshot & scene_snapshot,
    const MtcGraspProfileConfig & config,
    manipulation::workflow::TransferExecutionContext & context) = 0;

  virtual void request_stop() noexcept = 0;
};

/** Generic port adapter selecting an object profile's named MTC grasp profile. */
class MtcGraspProvider final : public manipulation::workflow::GraspProvider
{
public:
  MtcGraspProvider(
    MtcGraspProfiles profiles,
    std::unique_ptr<MtcSnapshotPlanner> planner);

  [[nodiscard]] manipulation::workflow::GraspProviderResult generate(
    const manipulation::workflow::ObjectObservation & observation,
    const manipulation::workflow::ObjectProfile & profile,
    const manipulation::workflow::PlanningSceneSnapshot & scene_snapshot,
    manipulation::workflow::TransferExecutionContext & context) override;

  void request_stop() noexcept override;

private:
  MtcGraspProfiles profiles_;
  std::unique_ptr<MtcSnapshotPlanner> planner_;
  std::atomic<std::uint64_t> stop_generation_{0U};
};

/** Production planner that consumes only a caller-owned local scene copy. */
class FixedStateMtcSnapshotPlanner final : public MtcSnapshotPlanner
{
public:
  FixedStateMtcSnapshotPlanner(
    moveit::core::RobotModelConstPtr robot_model,
    std::string output_link_frame);

  FixedStateMtcSnapshotPlanner(
    moveit::core::RobotModelConstPtr robot_model,
    std::string output_link_frame,
    detail::FixedStateMtcTaskOperations task_operations);

  [[nodiscard]] manipulation::workflow::GraspProviderResult plan(
    const manipulation::workflow::ObjectObservation & observation,
    const manipulation::workflow::ObjectProfile & profile,
    const manipulation::workflow::PlanningSceneSnapshot & scene_snapshot,
    const MtcGraspProfileConfig & config,
    manipulation::workflow::TransferExecutionContext & context) override;

  void request_stop() noexcept override;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  std::string output_link_frame_;
  detail::FixedStateMtcTaskOperations task_operations_;
  std::atomic<std::uint64_t> stop_generation_{0U};
};

namespace detail
{

/** Outcome of one blocking MTC plan call guarded by the production stop monitor. */
struct InterruptibleMtcPlanResult
{
  bool planned{false};
  bool stopped{false};
};

/**
 * Run a blocking MTC operation while forwarding cooperative stop exactly once.
 *
 * The callbacks are a narrow test seam around Task::plan()/Task::preempt();
 * production task construction and solution extraction remain unchanged.
 */
[[nodiscard]] InterruptibleMtcPlanResult run_interruptible_mtc_plan(
  std::function<bool()> plan,
  std::function<bool()> stop_requested,
  std::function<void()> preempt);

/** Copy a complete immutable message into a new, caller-owned PlanningScene. */
[[nodiscard]] planning_scene::PlanningScenePtr make_local_snapshot_scene(
  const moveit::core::RobotModelConstPtr & robot_model,
  const manipulation::workflow::PlanningSceneSnapshot & snapshot);

/**
 * Add one local-only object-centred frame aligned with the workflow frame.
 *
 * The observed object pose and collision geometry remain unchanged.  The
 * returned subframe exists only in the caller-owned scene copy and gives MTC
 * an explicit frame for workflow-relative sampling axes.
 */
[[nodiscard]] std::string add_workflow_aligned_grasp_frame(
  const planning_scene::PlanningScenePtr & local_scene,
  const std::string & object_id,
  const std::string & workflow_frame);

/** Build a task whose only initial-state source is stages::FixedState. */
[[nodiscard]] moveit::task_constructor::Task make_fixed_state_grasp_task(
  const planning_scene::PlanningScenePtr & local_scene,
  const std::string & grasp_reference_frame,
  const MtcGraspProfileConfig & config);

}  // namespace detail
}  // namespace franka_motion::workflow_v2

#endif  // WORKFLOW_V2__MTC_GRASP_PROVIDER_HPP_
