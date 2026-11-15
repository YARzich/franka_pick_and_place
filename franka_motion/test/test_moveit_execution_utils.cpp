// Responsibility: Verifies MoveIt current-state readiness, retries, and execution failures.

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "franka_motion/moveit_execution_utils.hpp"

namespace
{
using moveit::core::MoveItErrorCode;

class RclcppTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

class FakeMoveGroup
{
public:
  struct Plan {};

  bool current_state_available{true};
  bool stamped_pose_target_result{true};
  std::vector<MoveItErrorCode> plan_results{MoveItErrorCode::SUCCESS};
  std::vector<MoveItErrorCode> execute_plan_results{MoveItErrorCode::SUCCESS};
  std::vector<MoveItErrorCode> execute_trajectory_results{MoveItErrorCode::SUCCESS};
  std::vector<double> cartesian_fractions{1.0};

  int current_state_calls{0};
  int start_state_calls{0};
  int pose_target_calls{0};
  int stamped_pose_target_calls{0};
  int clear_pose_targets_calls{0};
  int plan_calls{0};
  int execute_plan_calls{0};
  int execute_trajectory_calls{0};
  int stop_calls{0};
  int joint_target_calls{0};
  int cartesian_calls{0};
  std::string last_end_effector_link;
  std::function<void()> on_plan;

  bool getCurrentState(double timeout_sec)
  {
    static_cast<void>(timeout_sec);
    ++current_state_calls;
    return current_state_available;
  }

  void setStartStateToCurrentState()
  {
    ++start_state_calls;
  }

  void setPoseTarget(const geometry_msgs::msg::Pose &)
  {
    ++pose_target_calls;
  }

  bool setPoseTarget(
    const geometry_msgs::msg::PoseStamped &,
    const std::string & end_effector_link)
  {
    ++stamped_pose_target_calls;
    last_end_effector_link = end_effector_link;
    return stamped_pose_target_result;
  }

  void clearPoseTargets()
  {
    ++clear_pose_targets_calls;
  }

  MoveItErrorCode plan(Plan &)
  {
    ++plan_calls;
    if (on_plan) {
      on_plan();
    }
    return popOrLast(plan_results);
  }

  MoveItErrorCode execute(const Plan &)
  {
    ++execute_plan_calls;
    return popOrLast(execute_plan_results);
  }

  MoveItErrorCode execute(const moveit_msgs::msg::RobotTrajectory &)
  {
    ++execute_trajectory_calls;
    return popOrLast(execute_trajectory_results);
  }

  void stop()
  {
    ++stop_calls;
  }

  void setJointValueTarget(const std::vector<double> &)
  {
    ++joint_target_calls;
  }

  double computeCartesianPath(
    const std::vector<geometry_msgs::msg::Pose> &,
    double,
    moveit_msgs::msg::RobotTrajectory &)
  {
    ++cartesian_calls;
    if (cartesian_fractions.empty()) {
      return 1.0;
    }
    const double fraction = cartesian_fractions.front();
    if (cartesian_fractions.size() > 1) {
      cartesian_fractions.erase(cartesian_fractions.begin());
    }
    return fraction;
  }

private:
  static MoveItErrorCode popOrLast(std::vector<MoveItErrorCode> & values)
  {
    if (values.empty()) {
      return MoveItErrorCode::SUCCESS;
    }
    const auto value = values.front();
    if (values.size() > 1) {
      values.erase(values.begin());
    }
    return value;
  }
};

void markFresh(franka_motion::FrankaJointStateMonitor & monitor)
{
  sensor_msgs::msg::JointState msg;
  msg.name = {"panda_joint1"};
  msg.position = {0.0};
  monitor.update(msg);
}
}  // namespace

TEST_F(RclcppTest, WaitForCurrentStateFailsWithoutFreshJointState)
{
  FakeMoveGroup move_group;
  franka_motion::FrankaJointStateMonitor monitor;

  EXPECT_FALSE(
    franka_motion::waitForCurrentState(
      move_group,
      monitor,
      rclcpp::get_logger("test_moveit_execution_utils"),
      "missing_state",
      std::chrono::milliseconds(0)));
  EXPECT_EQ(move_group.current_state_calls, 0);
}

TEST_F(RclcppTest, WaitForCurrentStateHonorsApplicationStop)
{
  FakeMoveGroup move_group;
  franka_motion::FrankaJointStateMonitor monitor;
  markFresh(monitor);

  EXPECT_FALSE(
    franka_motion::waitForCurrentState(
      move_group,
      monitor,
      rclcpp::get_logger("test_moveit_execution_utils"),
      "stopped",
      std::chrono::seconds(1),
      std::chrono::seconds(1),
      []() {return true;}));
  EXPECT_EQ(move_group.current_state_calls, 0);
}

TEST_F(RclcppTest, PlanAndExecutePoseRetriesPlanningFailure)
{
  FakeMoveGroup move_group;
  move_group.plan_results = {MoveItErrorCode::PLANNING_FAILED, MoveItErrorCode::SUCCESS};
  franka_motion::FrankaJointStateMonitor monitor;
  markFresh(monitor);

  EXPECT_TRUE(
    franka_motion::planAndExecutePose(
      move_group,
      monitor,
      rclcpp::get_logger("test_moveit_execution_utils"),
      geometry_msgs::msg::Pose{},
      "pose",
      2,
      std::chrono::seconds(1)));

  EXPECT_EQ(move_group.plan_calls, 2);
  EXPECT_EQ(move_group.execute_plan_calls, 1);
  EXPECT_EQ(move_group.current_state_calls, 2);
  EXPECT_EQ(move_group.start_state_calls, 2);
  EXPECT_EQ(move_group.pose_target_calls, 2);
  EXPECT_EQ(move_group.clear_pose_targets_calls, 2);
  EXPECT_EQ(move_group.stop_calls, 1);
}

TEST_F(RclcppTest, StopAfterPlanningPreventsExecution)
{
  FakeMoveGroup move_group;
  bool stop_requested = false;
  move_group.on_plan = [&stop_requested]() {stop_requested = true;};
  franka_motion::FrankaJointStateMonitor monitor;
  markFresh(monitor);

  EXPECT_FALSE(
    franka_motion::planAndExecuteJoints(
      move_group,
      monitor,
      rclcpp::get_logger("test_moveit_execution_utils"),
      std::vector<double>{0.0, 1.0},
      "stopped_after_plan",
      2,
      std::chrono::seconds(1),
      [&stop_requested]() {return stop_requested;}));

  EXPECT_EQ(move_group.plan_calls, 1);
  EXPECT_EQ(move_group.execute_plan_calls, 0);
  EXPECT_EQ(move_group.stop_calls, 0);
}

TEST_F(RclcppTest, PlanAndExecuteHonorsExplicitCurrentStateDeadline)
{
  FakeMoveGroup move_group;
  franka_motion::FrankaJointStateMonitor monitor;
  markFresh(monitor);
  const auto started = std::chrono::steady_clock::now();

  EXPECT_FALSE(
    franka_motion::planAndExecuteJoints(
      move_group,
      monitor,
      rclcpp::get_logger("test_moveit_execution_utils"),
      std::vector<double>{0.0, 1.0},
      "bounded_state_wait",
      1,
      std::chrono::seconds(1),
      {},
      std::chrono::steady_clock::duration::zero()));

  EXPECT_LT(std::chrono::steady_clock::now() - started, std::chrono::milliseconds(500));
  EXPECT_EQ(move_group.current_state_calls, 0);
  EXPECT_EQ(move_group.plan_calls, 0);
}

TEST_F(RclcppTest, PlanAndExecuteStampedPoseStopsOnInvalidTarget)
{
  FakeMoveGroup move_group;
  move_group.stamped_pose_target_result = false;
  franka_motion::FrankaJointStateMonitor monitor;
  markFresh(monitor);

  EXPECT_FALSE(
    franka_motion::planAndExecutePose(
      move_group,
      monitor,
      rclcpp::get_logger("test_moveit_execution_utils"),
      geometry_msgs::msg::PoseStamped{},
      "panda_grasp_frame",
      "stamped_pose",
      2,
      std::chrono::seconds(1)));

  EXPECT_EQ(move_group.stamped_pose_target_calls, 1);
  EXPECT_EQ(move_group.current_state_calls, 1);
  EXPECT_EQ(move_group.start_state_calls, 1);
  EXPECT_EQ(move_group.clear_pose_targets_calls, 1);
  EXPECT_EQ(move_group.plan_calls, 0);
  EXPECT_EQ(move_group.execute_plan_calls, 0);
  EXPECT_EQ(move_group.stop_calls, 0);
  EXPECT_EQ(move_group.last_end_effector_link, "panda_grasp_frame");
}

TEST_F(RclcppTest, PlanAndExecuteCartesianFailsAfterLowFraction)
{
  FakeMoveGroup move_group;
  move_group.cartesian_fractions = {0.5, 0.5};
  franka_motion::FrankaJointStateMonitor monitor;
  markFresh(monitor);

  EXPECT_FALSE(
    franka_motion::planAndExecuteCartesian(
      move_group,
      monitor,
      rclcpp::get_logger("test_moveit_execution_utils"),
      geometry_msgs::msg::Pose{},
      "cartesian",
      2,
      std::chrono::seconds(1)));

  EXPECT_EQ(move_group.cartesian_calls, 2);
  EXPECT_EQ(move_group.current_state_calls, 2);
  EXPECT_EQ(move_group.start_state_calls, 2);
  EXPECT_EQ(move_group.execute_trajectory_calls, 0);
  EXPECT_EQ(move_group.stop_calls, 0);
}

TEST_F(RclcppTest, PlanAndExecuteCartesianRetriesExecutionFailure)
{
  FakeMoveGroup move_group;
  move_group.cartesian_fractions = {1.0, 1.0};
  move_group.execute_trajectory_results = {
    MoveItErrorCode::CONTROL_FAILED,
    MoveItErrorCode::SUCCESS};
  franka_motion::FrankaJointStateMonitor monitor;
  markFresh(monitor);

  EXPECT_TRUE(
    franka_motion::planAndExecuteCartesian(
      move_group,
      monitor,
      rclcpp::get_logger("test_moveit_execution_utils"),
      geometry_msgs::msg::Pose{},
      "cartesian",
      2,
      std::chrono::seconds(1)));

  EXPECT_EQ(move_group.cartesian_calls, 2);
  EXPECT_EQ(move_group.current_state_calls, 2);
  EXPECT_EQ(move_group.start_state_calls, 2);
  EXPECT_EQ(move_group.execute_trajectory_calls, 2);
  EXPECT_EQ(move_group.stop_calls, 2);
}

TEST_F(RclcppTest, PlanAndExecuteJointsRetriesExecutionFailure)
{
  FakeMoveGroup move_group;
  move_group.plan_results = {MoveItErrorCode::SUCCESS, MoveItErrorCode::SUCCESS};
  move_group.execute_plan_results = {
    MoveItErrorCode::CONTROL_FAILED,
    MoveItErrorCode::SUCCESS};
  franka_motion::FrankaJointStateMonitor monitor;
  markFresh(monitor);

  EXPECT_TRUE(
    franka_motion::planAndExecuteJoints(
      move_group,
      monitor,
      rclcpp::get_logger("test_moveit_execution_utils"),
      std::vector<double>{0.0, 1.0},
      "joints",
      2,
      std::chrono::seconds(1)));

  EXPECT_EQ(move_group.joint_target_calls, 2);
  EXPECT_EQ(move_group.current_state_calls, 2);
  EXPECT_EQ(move_group.start_state_calls, 2);
  EXPECT_EQ(move_group.plan_calls, 2);
  EXPECT_EQ(move_group.execute_plan_calls, 2);
  EXPECT_EQ(move_group.stop_calls, 2);
}
