// Responsibility: Defines pure provider and strategy ports for the resolved-transfer workflow.

#ifndef MANIPULATION__WORKFLOW__TRANSFER_WORKFLOW_PORTS_HPP_
#define MANIPULATION__WORKFLOW__TRANSFER_WORKFLOW_PORTS_HPP_

#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "manipulation/task/task_status.hpp"
#include "manipulation/workflow/transfer_execution_context.hpp"
#include "manipulation/workflow/transfer_workflow_types.hpp"

namespace manipulation::workflow
{

/** Typed success-or-failure value shared by pure provider boundaries. */
template<typename ValueT>
class ProviderResult final
{
public:
  [[nodiscard]] static ProviderResult success(ValueT value)
  {
    return ProviderResult(
      std::optional<ValueT>{std::move(value)}, manipulation::task::TaskErrorCode::kNone, {});
  }

  [[nodiscard]] static ProviderResult failure(
    const manipulation::task::TaskErrorCode error_code,
    std::string detail)
  {
    if (error_code == manipulation::task::TaskErrorCode::kNone) {
      throw std::invalid_argument("provider failure must carry a non-success error code");
    }
    return ProviderResult(std::nullopt, error_code, std::move(detail));
  }

  [[nodiscard]] bool succeeded() const noexcept
  {
    return value_.has_value() && error_code_ == manipulation::task::TaskErrorCode::kNone;
  }

  [[nodiscard]] const std::optional<ValueT> & value() const noexcept
  {
    return value_;
  }

  [[nodiscard]] manipulation::task::TaskErrorCode error_code() const noexcept
  {
    return error_code_;
  }

  [[nodiscard]] const std::string & detail() const noexcept
  {
    return detail_;
  }

private:
  ProviderResult(
    std::optional<ValueT> value,
    const manipulation::task::TaskErrorCode error_code,
    std::string detail)
  : value_(std::move(value)), error_code_(error_code), detail_(std::move(detail))
  {
  }

  std::optional<ValueT> value_;
  manipulation::task::TaskErrorCode error_code_;
  std::string detail_;
};

using ObservationProviderResult = ProviderResult<ObjectObservation>;
using GraspProviderResult = ProviderResult<std::vector<GraspCandidate>>;

/** Obtain a fresh stamped observation for one catalog-resolved object. */
class ObservationProvider
{
public:
  virtual ~ObservationProvider() = default;

  [[nodiscard]] virtual ObservationProviderResult observe(
    const ObjectInstance & instance,
    const ObjectProfile & profile,
    TransferExecutionContext & context) = 0;

  /// Non-blocking propagation to any provider-owned external operation.
  virtual void request_stop() noexcept = 0;
};

/** Generate ranked grasps from immutable observation, profile, and scene evidence. */
class GraspProvider
{
public:
  virtual ~GraspProvider() = default;

  [[nodiscard]] virtual GraspProviderResult generate(
    const ObjectObservation & observation,
    const ObjectProfile & profile,
    const PlanningSceneSnapshot & scene_snapshot,
    TransferExecutionContext & context) = 0;

  /// Non-blocking propagation to any provider-owned external operation.
  virtual void request_stop() noexcept = 0;
};

/** Execute one already-resolved transfer through constructor-injected backend ports. */
class TransferExecutionStrategy
{
public:
  virtual ~TransferExecutionStrategy() = default;

  [[nodiscard]] virtual std::string_view id() const noexcept = 0;

  [[nodiscard]] virtual manipulation::task::TaskOutcome execute(
    const ResolvedTransferTask & task,
    TransferExecutionContext & context) = 0;

  /// Non-blocking propagation to strategy-owned physical/backend operations.
  virtual void request_stop() noexcept = 0;
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__TRANSFER_WORKFLOW_PORTS_HPP_
