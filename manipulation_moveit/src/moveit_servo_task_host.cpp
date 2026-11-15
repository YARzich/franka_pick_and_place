// Responsibility: Composes the standalone ServoToPose task and MoveIt Servo adapter.

#include "manipulation_moveit/moveit_servo_task_host.hpp"

#include <memory>
#include <string>
#include <utility>

#include "manipulation/skill/capability_set.hpp"
#include "manipulation/skill/resource_claim.hpp"
#include "manipulation/skill/resource_registry.hpp"
#include "manipulation/skill/skill_descriptor.hpp"
#include "manipulation/skill/standard_capabilities.hpp"
#include "manipulation/workflow/skill_runtime.hpp"
#include "manipulation_moveit/servo/moveit_servo_ros_adapter.hpp"
#include "manipulation_moveit/servo/servo_to_pose_action_server.hpp"
#include "manipulation_moveit/servo/servo_to_pose_port.hpp"

namespace manipulation::moveit
{
namespace
{

manipulation::skill::CapabilitySet servo_capabilities()
{
  using manipulation::skill::capability::kExactStampTransform;
  using manipulation::skill::capability::kServoPose;
  return manipulation::skill::CapabilitySet{
    std::string{kServoPose}, std::string{kExactStampTransform}};
}

std::shared_ptr<const manipulation::workflow::SkillRuntime> make_servo_runtime(
  const LoadedMoveItServoProcessConfig & config,
  const manipulation::skill::PresetIdentity & deployment,
  const manipulation::skill::CapabilitySet & available,
  const std::shared_ptr<manipulation::skill::ResourceRegistry> & registry)
{
  const auto descriptor = manipulation::skill::SkillDescriptor::create(
    "servo/to_pose",
    config.preset_identity,
    servo_capabilities(),
    {manipulation::skill::ResourceClaim{config.arm_resource_id}},
    deployment);
  auto runtime = manipulation::workflow::SkillRuntime::create(
    descriptor, available, registry);
  return std::make_shared<const manipulation::workflow::SkillRuntime>(
    std::move(runtime));
}

ServoToPoseActionServerConfig with_runtime(
  ServoToPoseActionServerConfig config,
  std::shared_ptr<const manipulation::workflow::SkillRuntime> runtime)
{
  config.skill_runtime = std::move(runtime);
  return config;
}

}  // namespace

class MoveItServoTaskHost::Impl final
{
public:
  Impl(rclcpp::Node & node, LoadedMoveItServoProcessConfig config)
  : config_(std::move(config)),
    registry_(std::make_shared<manipulation::skill::ResourceRegistry>()),
    deployment_(make_moveit_servo_deployment_identity(config_)),
    available_(servo_capabilities()),
    runtime_(make_servo_runtime(config_, deployment_, available_, registry_)),
    adapter_(node, config_.adapter),
    port_(adapter_),
    server_(node, port_, with_runtime(config_.action_server, runtime_))
  {
    RCLCPP_INFO(
      node.get_logger(),
      "MoveIt Servo task server ready: preset=%s deployment=%s",
      config_.preset_identity.preset_id().c_str(),
      deployment_.canonical_digest().c_str());
  }

private:
  // Destruction order is deliberate: the facade dies before borrowed backends.
  LoadedMoveItServoProcessConfig config_;
  std::shared_ptr<manipulation::skill::ResourceRegistry> registry_;
  manipulation::skill::PresetIdentity deployment_;
  manipulation::skill::CapabilitySet available_;
  std::shared_ptr<const manipulation::workflow::SkillRuntime> runtime_;
  MoveItServoRosAdapter adapter_;
  MoveItServoTaskPort port_;
  ServoToPoseActionServer server_;
};

MoveItServoTaskHost::MoveItServoTaskHost(
  rclcpp::Node & node,
  LoadedMoveItServoProcessConfig config)
: impl_(std::make_unique<Impl>(node, std::move(config)))
{
}

MoveItServoTaskHost::~MoveItServoTaskHost() = default;

}  // namespace manipulation::moveit
