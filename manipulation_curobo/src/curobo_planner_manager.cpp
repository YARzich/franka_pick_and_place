// Responsibility: Registers the cuRobo MoveIt plugin and creates validated per-request contexts.

#include "manipulation_curobo/curobo_planner_manager.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "manipulation_curobo/child_process.hpp"
#include "manipulation_curobo/curobo_planning_context.hpp"
#include "manipulation_curobo/temporary_file.hpp"
#include "package_resource.hpp"

namespace manipulation_curobo
{
namespace
{

using namespace std::chrono_literals;
constexpr auto kReadinessProbeDeadline = 30s;

struct ReadinessProbe
{
  std::string workspace_frame;
  std::string ee_link;
  std::string tensor_device;
};

std::string parameterName(
  const std::string & parameter_namespace,
  const std::string & name)
{
  const auto begin = parameter_namespace.find_first_not_of('.');
  if (begin == std::string::npos) {
    return name;
  }
  const auto end = parameter_namespace.find_last_not_of('.');
  return parameter_namespace.substr(begin, end - begin + 1U) + "." + name;
}

bool parseReadinessProbe(
  const std::filesystem::path & path, ReadinessProbe & probe,
  std::string & error)
{
  try {
    const YAML::Node root = YAML::LoadFile(path.string());
    if (!root.IsMap() || !root["success"] || !root["success"].as<bool>()) {
      error = root["message"] ? root["message"].as<std::string>() :
        "cuRobo readiness probe returned no success evidence";
      return false;
    }
    probe.workspace_frame = root["workspace_frame"].as<std::string>("");
    probe.ee_link = root["ee_link"].as<std::string>("");
    probe.tensor_device = root["tensor_device"].as<std::string>("");
    if (probe.workspace_frame.empty() || probe.ee_link.empty() || probe.tensor_device.empty()) {
      error = "cuRobo readiness probe returned incomplete configuration evidence";
      return false;
    }
    return true;
  } catch (const std::exception & exception) {
    error = std::string{"cannot parse cuRobo readiness evidence: "} + exception.what();
    return false;
  }
}

}  // namespace

CuRoboPlannerManager::CuRoboPlannerManager()
: logger_(rclcpp::get_logger("manipulation_curobo.planner"))
{
}

bool CuRoboPlannerManager::initialize(
  const moveit::core::RobotModelConstPtr & model,
  const rclcpp::Node::SharedPtr & node,
  const std::string & parameter_namespace)
{
  admission_gate_.reset();
  workspace_frame_.clear();
  robot_model_.reset();
  node_.reset();
  if (!model || !node) {
    RCLCPP_ERROR(logger_, "cuRobo planner requires RobotModel and ROS node");
    return false;
  }
  python_executable_ = "python3";
  curobo_ee_link_.clear();
  motion_gen_config_.clear();
  std::int64_t max_concurrent_plans = 1;
  try {
    const std::filesystem::path package_share =
      ament_index_cpp::get_package_share_directory("manipulation_curobo");
    planner_script_ = (package_share / "scripts" / "curobo_motion_bridge.py").string();

    const auto ensure_parameter = [&node, &parameter_namespace](
      const std::string & name, auto & value) {
        const std::string full_name = parameterName(parameter_namespace, name);
        if (!node->has_parameter(full_name)) {
          node->declare_parameter(full_name, value);
        }
        (void)node->get_parameter(full_name, value);
      };
    ensure_parameter("python_executable", python_executable_);
    ensure_parameter("bridge_script", planner_script_);
    ensure_parameter("motion_gen_config", motion_gen_config_);
    ensure_parameter("ee_link", curobo_ee_link_);
    ensure_parameter("max_concurrent_plans", max_concurrent_plans);
    motion_gen_config_ = detail::resolvePackageResource(motion_gen_config_);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(logger_, "Cannot initialize cuRobo planner resources: %s", exception.what());
    return false;
  }

  if (python_executable_.empty() || planner_script_.empty() || motion_gen_config_.empty() ||
    curobo_ee_link_.empty())
  {
    RCLCPP_ERROR(logger_, "cuRobo planner parameters must not be empty");
    return false;
  }
  if (!std::filesystem::is_regular_file(planner_script_)) {
    RCLCPP_ERROR(logger_, "cuRobo bridge does not exist: %s", planner_script_.c_str());
    return false;
  }
  if (!std::filesystem::is_regular_file(motion_gen_config_)) {
    RCLCPP_ERROR(
      logger_, "cuRobo MotionGen config does not exist: %s", motion_gen_config_.c_str());
    return false;
  }
  if (max_concurrent_plans < 1 ||
    static_cast<std::uint64_t>(max_concurrent_plans) >
    static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
  {
    RCLCPP_ERROR(logger_, "cuRobo max_concurrent_plans must be a positive integer");
    return false;
  }

  try {
    curobo::TemporaryFile probe_file{"curobo_probe"};
    curobo::ChildProcess probe_process;
    const std::vector<std::string> arguments{
      python_executable_, planner_script_, "--probe-config", motion_gen_config_,
      "--output", probe_file.path().string()};
    const bool started = probe_process.start(arguments);
    const auto process_result = probe_process.waitUntil(
      curobo::ChildProcess::Clock::now() + kReadinessProbeDeadline);
    if (!started || !process_result.succeeded()) {
      RCLCPP_ERROR(
        logger_, "cuRobo readiness probe could not run (outcome=%d, exit=%d): %s",
        static_cast<int>(process_result.outcome), process_result.exit_code,
        process_result.detail.c_str());
      return false;
    }
    ReadinessProbe probe;
    std::string error;
    if (!parseReadinessProbe(probe_file.path(), probe, error)) {
      RCLCPP_ERROR(logger_, "cuRobo backend is not ready: %s", error.c_str());
      return false;
    }
    if (probe.ee_link != curobo_ee_link_) {
      RCLCPP_ERROR(
        logger_, "cuRobo ee_link mismatch: pipeline='%s', MotionGen='%s'",
        curobo_ee_link_.c_str(), probe.ee_link.c_str());
      return false;
    }
    if (!model->hasLinkModel(curobo_ee_link_)) {
      RCLCPP_ERROR(
        logger_, "cuRobo ee_link is absent from MoveIt RobotModel: %s",
        curobo_ee_link_.c_str());
      return false;
    }
    workspace_frame_ = std::move(probe.workspace_frame);
    RCLCPP_INFO(
      logger_, "cuRobo readiness probe passed for workspace '%s' on '%s'",
      workspace_frame_.c_str(), probe.tensor_device.c_str());
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(logger_, "Cannot run cuRobo readiness probe: %s", exception.what());
    return false;
  }
  auto admission_gate = std::make_shared<curobo::PlanningAdmissionGate>(
    static_cast<std::size_t>(max_concurrent_plans));
  robot_model_ = model;
  node_ = node;
  admission_gate_ = std::move(admission_gate);
  return true;
}

planning_interface::PlanningContextPtr CuRoboPlannerManager::getPlanningContext(
  const planning_scene::PlanningSceneConstPtr & planning_scene,
  const planning_interface::MotionPlanRequest & request,
  moveit_msgs::msg::MoveItErrorCodes & error_code) const
{
  if (!planning_scene || !canServiceRequest(request) || !admission_gate_) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return {};
  }
  auto context = std::make_shared<CuRoboPlanningContext>(
    "curobo_context", request.group_name, robot_model_, python_executable_,
    planner_script_, motion_gen_config_, workspace_frame_, curobo_ee_link_,
    admission_gate_, logger_);
  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(request);
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return context;
}

bool CuRoboPlannerManager::canServiceRequest(
  const planning_interface::MotionPlanRequest & request) const
{
  return robot_model_ && robot_model_->getJointModelGroup(request.group_name) != nullptr &&
         request.goal_constraints.size() == 1U;
}

void CuRoboPlannerManager::getPlanningAlgorithms(
  std::vector<std::string> & algorithms) const
{
  algorithms.push_back("curobo");
}

std::string CuRoboPlannerManager::getDescription() const
{
  return "optional cuRobo MotionGen reference planner";
}

}  // namespace manipulation_curobo

// Register the reference backend under its robot-independent plugin identity.
PLUGINLIB_EXPORT_CLASS(manipulation_curobo::CuRoboPlannerManager,
  planning_interface::PlannerManager)
