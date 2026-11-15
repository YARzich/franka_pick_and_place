// Responsibility: Declares the V2 readiness gate over typed health and injected backend probes.

#ifndef MANIPULATION_MOVEIT__ROS_PICK_LIFT_PLACE_HEALTH_PORT_HPP_
#define MANIPULATION_MOVEIT__ROS_PICK_LIFT_PLACE_HEALTH_PORT_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "manipulation_moveit/pick_lift_place_ports.hpp"

namespace manipulation::moveit
{

using PickLiftPlaceReadinessProbe = std::function<PickLiftPlaceReadiness()>;

/** Required read-only probes composing all V2 production dependencies. */
struct RosPickLiftPlaceReadinessProbes
{
  PickLiftPlaceReadinessProbe planning;
  PickLiftPlaceReadinessProbe trajectory;
  PickLiftPlaceReadinessProbe gripper;
  PickLiftPlaceReadinessProbe scene;
};

/** Deployment topic only; freshness and wait slicing are private invariants. */
struct RosPickLiftPlaceHealthPortConfig
{
  std::string topic{"/system_health_monitor/health"};
};

void validateRosPickLiftPlaceHealthPortConfig(
  const RosPickLiftPlaceHealthPortConfig & config,
  const RosPickLiftPlaceReadinessProbes & probes);

/**
 * Reliable transient-local SystemHealth gate with a private freshness watchdog.
 *
 * `wait_until_ready()` has no deadline. It reports typed health waiting and
 * continues until the latest locally fresh snapshot allows motion and every
 * injected backend probe is ready, or until cancel/shutdown/final error.
 */
class RosPickLiftPlaceHealthPort final : public IPickLiftPlaceHealthPort
{
public:
  RosPickLiftPlaceHealthPort(
    rclcpp::Node & node,
    RosPickLiftPlaceHealthPortConfig config,
    RosPickLiftPlaceReadinessProbes probes);
  ~RosPickLiftPlaceHealthPort() override;

  RosPickLiftPlaceHealthPort(const RosPickLiftPlaceHealthPort &) = delete;
  RosPickLiftPlaceHealthPort & operator=(const RosPickLiftPlaceHealthPort &) = delete;
  RosPickLiftPlaceHealthPort(RosPickLiftPlaceHealthPort &&) = delete;
  RosPickLiftPlaceHealthPort & operator=(RosPickLiftPlaceHealthPort &&) = delete;

  void begin_goal() noexcept override;
  [[nodiscard]] PickLiftPlaceReadiness cached_readiness() const;
  [[nodiscard]] PickLiftPlaceHealthResult wait_until_ready(
    manipulation::workflow::TransferExecutionContext & context) override;
  void request_stop() noexcept override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace manipulation::moveit

#endif  // MANIPULATION_MOVEIT__ROS_PICK_LIFT_PLACE_HEALTH_PORT_HPP_
