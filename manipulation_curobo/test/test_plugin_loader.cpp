// Responsibility: Verify the sole public cuRobo plugin ID instantiates and probes readiness.

#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "moveit/planning_interface/planning_interface.hpp"
#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit/utils/robot_model_test_utils.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include "manipulation_curobo/temporary_file.hpp"

namespace
{

class RosEnvironment final : public testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owns_context_ = true;
    }
  }

  void TearDown() override
  {
    if (owns_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

private:
  bool owns_context_{false};
};

[[maybe_unused]] const auto * const ros_environment =
  testing::AddGlobalTestEnvironment(new RosEnvironment);

using manipulation_curobo::curobo::TemporaryFile;
using namespace std::chrono_literals;

void writeText(const TemporaryFile & file, const std::string & text)
{
  std::ofstream stream(file.path(), std::ios::out | std::ios::trunc);
  ASSERT_TRUE(stream.is_open());
  stream << text;
  ASSERT_TRUE(stream.good());
}

std::shared_ptr<planning_interface::PlannerManager> loadPlanner()
{
  static pluginlib::ClassLoader<planning_interface::PlannerManager> loader(
    "moveit_core", "planning_interface::PlannerManager");
  return loader.createSharedInstance("manipulation_curobo/CuRoboPlanner");
}

std::string fakeBridgeSource(
  const std::string & workspace_frame,
  const std::filesystem::path & lifecycle_evidence = {})
{
  std::string source =
    "import sys\n"
    "import time\n"
    "import yaml\n"
    "output = sys.argv[sys.argv.index('--output') + 1]\n"
    "if '--probe-config' in sys.argv:\n"
    "    with open(output, 'w', encoding='utf-8') as stream:\n"
    "        yaml.safe_dump({'success': True, 'workspace_frame': '" + workspace_frame +
    "', 'ee_link': 'panda_hand', 'tensor_device': 'fake:cpu'}, stream)\n"
    "    raise SystemExit(0)\n"
    "input_path = sys.argv[sys.argv.index('--input') + 1]\n"
    "with open(input_path, encoding='utf-8') as stream:\n"
    "    request = yaml.safe_load(stream)\n";
  if (!lifecycle_evidence.empty()) {
    source +=
      "with open('" + lifecycle_evidence.string() + "', 'w', encoding='utf-8') as stream:\n"
      "    stream.write('accepted')\n"
      "time.sleep(60)\n";
  }
  source +=
    "names = request['joint_names']\n"
    "start = request['start_positions']\n"
    "point = {'positions': start, 'velocities': [0.0] * len(names), "
    "'accelerations': [0.0] * len(names)}\n"
    "response = {'success': True, 'planning_time': 0.125, 'trajectory': "
    "{'dt': 0.01, 'joint_names': names, 'points': [point, point]}}\n"
    "with open(output, 'w', encoding='utf-8') as stream:\n"
    "    yaml.safe_dump(response, stream)\n";
  return source;
}

planning_interface::MotionPlanRequest makeJointRequest(
  const moveit::core::JointModelGroup & group,
  const std::vector<double> & positions)
{
  planning_interface::MotionPlanRequest request;
  request.group_name = group.getName();
  request.allowed_planning_time = 0.25;
  request.goal_constraints.resize(1U);
  const auto & names = group.getVariableNames();
  for (std::size_t index = 0; index < names.size(); ++index) {
    moveit_msgs::msg::JointConstraint constraint;
    constraint.joint_name = names[index];
    constraint.position = positions[index];
    request.goal_constraints.front().joint_constraints.push_back(constraint);
  }
  return request;
}

planning_scene::PlanningScenePtr makePandaScene(
  const moveit::core::RobotModelPtr & model,
  const moveit::core::JointModelGroup & group,
  const std::vector<double> & positions)
{
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  auto & state = scene->getCurrentStateNonConst();
  state.setToDefaultValues();
  state.setJointGroupPositions(&group, positions);
  state.update();
  return scene;
}

std::shared_ptr<planning_interface::PlannerManager> initializeFakePlanner(
  const moveit::core::RobotModelPtr & model,
  const TemporaryFile & bridge,
  const TemporaryFile & config,
  const std::string & node_name,
  const std::string & parameter_namespace)
{
  auto node = std::make_shared<rclcpp::Node>(node_name);
  node->declare_parameter(
    parameter_namespace + ".python_executable", "/usr/bin/python3");
  node->declare_parameter(
    parameter_namespace + ".bridge_script", bridge.path().string());
  node->declare_parameter(
    parameter_namespace + ".motion_gen_config", config.path().string());
  node->declare_parameter(parameter_namespace + ".ee_link", "panda_hand");
  auto planner = loadPlanner();
  if (!planner->initialize(model, node, parameter_namespace)) {
    return {};
  }
  return planner;
}

std::string readText(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  return std::string{
    std::istreambuf_iterator<char>{stream}, std::istreambuf_iterator<char>{}};
}

}  // namespace

TEST(CuRoboPluginLoader, InstantiatesCanonicalId)
{
  const auto canonical = loadPlanner();
  EXPECT_NE(canonical, nullptr);
}

TEST(CuRoboPluginLoader, InitializeRunsReadinessProbeBeforeServingRequests)
{
  const auto model = moveit::core::loadTestingRobotModel("panda");
  ASSERT_NE(model, nullptr);
  TemporaryFile bridge{"curobo_fake_probe"};
  TemporaryFile config{"curobo_fake_config"};
  writeText(
    bridge,
    "import sys\n"
    "output = sys.argv[sys.argv.index('--output') + 1]\n"
    "with open(output, 'w', encoding='utf-8') as stream:\n"
    "    stream.write('success: true\\nworkspace_frame: world\\n'"
    "                 'ee_link: panda_hand\\ntensor_device: cuda:0\\n')\n");
  writeText(config, "motion_gen: {}\n");

  auto node = std::make_shared<rclcpp::Node>("curobo_probe_success_test");
  node->declare_parameter("probe.python_executable", "/usr/bin/python3");
  node->declare_parameter("probe.bridge_script", bridge.path().string());
  node->declare_parameter("probe.motion_gen_config", config.path().string());
  node->declare_parameter("probe.ee_link", "panda_hand");
  const auto planner = loadPlanner();

  ASSERT_TRUE(planner->initialize(model, node, "probe"));
  planning_interface::MotionPlanRequest request;
  request.group_name = "panda_arm";
  request.goal_constraints.resize(1U);
  EXPECT_TRUE(planner->canServiceRequest(request));
}

TEST(CuRoboPluginLoader, InitializeFailsBeforeAdmissionWhenExecutableIsUnavailable)
{
  const auto model = moveit::core::loadTestingRobotModel("panda");
  ASSERT_NE(model, nullptr);
  TemporaryFile bridge{"curobo_fake_probe"};
  TemporaryFile config{"curobo_fake_config"};
  writeText(bridge, "raise SystemExit(0)\n");
  writeText(config, "motion_gen: {}\n");

  auto node = std::make_shared<rclcpp::Node>("curobo_probe_failure_test");
  node->declare_parameter("probe.python_executable", "/definitely/not/python");
  node->declare_parameter("probe.bridge_script", bridge.path().string());
  node->declare_parameter("probe.motion_gen_config", config.path().string());
  node->declare_parameter("probe.ee_link", "panda_hand");
  const auto planner = loadPlanner();

  EXPECT_FALSE(planner->initialize(model, node, "probe"));
  planning_interface::MotionPlanRequest request;
  request.group_name = "panda_arm";
  request.goal_constraints.resize(1U);
  EXPECT_FALSE(planner->canServiceRequest(request));
}

TEST(CuRoboPluginLoader, PlansThroughTheCanonicalMoveItContextWithFakeRuntime)
{
  const auto model = moveit::core::loadTestingRobotModel("panda");
  ASSERT_NE(model, nullptr);
  const auto * group = model->getJointModelGroup("panda_arm");
  ASSERT_NE(group, nullptr);
  const std::vector<double> positions{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
  ASSERT_EQ(positions.size(), group->getVariableCount());

  TemporaryFile bridge{"curobo_fake_plan"};
  TemporaryFile config{"curobo_fake_config"};
  writeText(bridge, fakeBridgeSource(model->getModelFrame()));
  writeText(config, "motion_gen: {}\n");
  const auto planner = initializeFakePlanner(
    model, bridge, config, "curobo_context_success_test", "fake_success");
  ASSERT_NE(planner, nullptr);

  const auto scene = makePandaScene(model, *group, positions);
  const auto request = makeJointRequest(*group, positions);
  moveit_msgs::msg::MoveItErrorCodes context_error;
  const auto context = planner->getPlanningContext(scene, request, context_error);
  ASSERT_NE(context, nullptr);
  ASSERT_EQ(context_error.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

  planning_interface::MotionPlanResponse response;
  context->solve(response);

  ASSERT_EQ(response.error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    << response.error_code.message;
  ASSERT_NE(response.trajectory, nullptr);
  EXPECT_EQ(response.trajectory->getWayPointCount(), 2U);
  EXPECT_DOUBLE_EQ(response.planning_time, 0.125);
}

TEST(CuRoboPluginLoader, CancelInterruptsAnAcceptedFakeRuntimeAndReapsIt)
{
  const auto model = moveit::core::loadTestingRobotModel("panda");
  ASSERT_NE(model, nullptr);
  const auto * group = model->getJointModelGroup("panda_arm");
  ASSERT_NE(group, nullptr);
  const std::vector<double> positions{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
  ASSERT_EQ(positions.size(), group->getVariableCount());

  TemporaryFile bridge{"curobo_fake_cancel"};
  TemporaryFile config{"curobo_fake_config"};
  TemporaryFile lifecycle_evidence{"curobo_fake_lifecycle"};
  writeText(
    bridge, fakeBridgeSource(model->getModelFrame(), lifecycle_evidence.path()));
  writeText(config, "motion_gen: {}\n");
  const auto planner = initializeFakePlanner(
    model, bridge, config, "curobo_context_cancel_test", "fake_cancel");
  ASSERT_NE(planner, nullptr);

  const auto scene = makePandaScene(model, *group, positions);
  const auto request = makeJointRequest(*group, positions);
  moveit_msgs::msg::MoveItErrorCodes context_error;
  const auto context = planner->getPlanningContext(scene, request, context_error);
  ASSERT_NE(context, nullptr);

  planning_interface::MotionPlanResponse response;
  auto solve = std::async(std::launch::async, [&context, &response]() {
        context->solve(response);
    });
  const auto evidence_deadline = std::chrono::steady_clock::now() + 3s;
  while (readText(lifecycle_evidence.path()) != "accepted" &&
    std::chrono::steady_clock::now() < evidence_deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  const bool accepted = readText(lifecycle_evidence.path()) == "accepted";
  if (!accepted) {
    (void)context->terminate();
  }
  ASSERT_TRUE(accepted);

  EXPECT_TRUE(context->terminate());
  ASSERT_EQ(solve.wait_for(3s), std::future_status::ready);
  solve.get();

  EXPECT_EQ(response.error_code.val, moveit_msgs::msg::MoveItErrorCodes::PREEMPTED);
  EXPECT_EQ(response.error_code.source, "manipulation_curobo");
  EXPECT_EQ(readText(lifecycle_evidence.path()), "accepted");
}
