// Responsibility: Synchronize the Franka Isaac physical attachment with PlanningScene mutations.

#ifndef WORKFLOW_V2__ISAAC_PHYSICAL_SCENE_PORT_HPP_
#define WORKFLOW_V2__ISAAC_PHYSICAL_SCENE_PORT_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "manipulation_moveit/pick_lift_place_ports.hpp"

namespace franka_motion::workflow_v2
{

/**
 * Object-scoped simulator coupling owned by the Franka reference application.
 *
 * An empty service name disables the simulator-specific barrier and preserves
 * the ordinary Gazebo/hardware scene path. A configured endpoint is scoped to
 * exactly one catalog object; other objects fail closed before any mutation.
 */
struct IsaacPhysicalScenePortConfig
{
  std::string service_name;
  std::string object_id{"cube"};
};

/** Reject malformed optional endpoint or object identity. */
void validateIsaacPhysicalScenePortConfig(
  const IsaacPhysicalScenePortConfig & config);

/**
 * Order a confirmed Isaac fixed-joint mutation before the matching scene diff.
 *
 * The SetBool response is the physical activation barrier. Endpoint discovery
 * and response waits have no semantic timeout and are interruptible by the
 * goal lifecycle. Once a request may have crossed the transport boundary, an
 * interrupted or malformed result is reported as state-uncertain.
 */
class IsaacPhysicalScenePort final
  : public manipulation::moveit::IPickLiftPlaceScenePort
{
public:
  IsaacPhysicalScenePort(
    rclcpp::Node & node,
    manipulation::moveit::IPickLiftPlaceScenePort & scene,
    IsaacPhysicalScenePortConfig config,
    std::function<manipulation::moveit::PickLiftPlaceReadiness()>
    scene_readiness);
  ~IsaacPhysicalScenePort() override;

  IsaacPhysicalScenePort(const IsaacPhysicalScenePort &) = delete;
  IsaacPhysicalScenePort & operator=(const IsaacPhysicalScenePort &) = delete;
  IsaacPhysicalScenePort(IsaacPhysicalScenePort &&) = delete;
  IsaacPhysicalScenePort & operator=(IsaacPhysicalScenePort &&) = delete;

  /** Combined raw-scene and optional physical-service readiness evidence. */
  [[nodiscard]] manipulation::moveit::PickLiftPlaceReadiness
  cached_readiness() const;

  void begin_goal() override;

  [[nodiscard]] manipulation::moveit::CoordinatedSceneResult revalidate(
    const manipulation::moveit::StrategySceneRevalidationRequest & request,
    manipulation::workflow::TransferExecutionContext & context) override;

  [[nodiscard]] manipulation::moveit::CoordinatedSceneResult attach(
    const manipulation::moveit::StrategySceneAttachRequest & request,
    manipulation::workflow::TransferExecutionContext & context) override;

  [[nodiscard]] manipulation::moveit::CoordinatedSceneResult detach(
    const manipulation::moveit::StrategySceneDetachRequest & request,
    manipulation::workflow::TransferExecutionContext & context) override;

  void request_stop() noexcept override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace franka_motion::workflow_v2

#endif  // WORKFLOW_V2__ISAAC_PHYSICAL_SCENE_PORT_HPP_
