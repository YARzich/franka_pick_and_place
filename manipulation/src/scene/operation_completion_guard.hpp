// Responsibility: Guarantees that an admitted scene operation receives exactly one terminal
//   registry update.

#ifndef SCENE__OPERATION_COMPLETION_GUARD_HPP_
#define SCENE__OPERATION_COMPLETION_GUARD_HPP_

#include <string>
#include <utility>

#include "manipulation/scene/coordinator_utils.hpp"
#include "manipulation/scene/scene_lease_registry.hpp"

namespace manipulation::scene
{

/**
 * Ensures every authorized registry ticket receives a terminal outcome.
 *
 * The guard borrows the registry and must not outlive it. It is confined to one
 * scene-I/O callback; asynchronous ownership must be transferred explicitly
 * with `release_to_async_completion()`. Destruction reports internal failure
 * best-effort and never throws.
 */
class OperationCompletionGuard final
{
public:
  OperationCompletionGuard(SceneLeaseRegistry & registry, std::string ticket)
  : registry_(registry), ticket_(std::move(ticket))
  {
  }

  ~OperationCompletionGuard()
  {
    if (!active_) {
      return;
    }
    try {
      registry_.complete_operation(
        ticket_,
        make_backend_operation_result(
          BackendCallOutcome::kInternalError,
          "coordinator abandoned an authorized backend operation"));
    } catch (...) {
      // The registry is non-throwing for valid tickets. This last boundary
      // prevents an unexpected allocation failure from terminating unwinding.
    }
  }

  OperationCompletionGuard(const OperationCompletionGuard &) = delete;
  OperationCompletionGuard & operator=(const OperationCompletionGuard &) = delete;

  /// Complete the ticket exactly once and disable destructor fallback.
  RegistryStatus complete(const BackendOperationResult & result)
  {
    const auto status = registry_.complete_operation(ticket_, result);
    active_ = false;
    return status;
  }

  /// Transfer completion responsibility to the coordinator's late-response tracker.
  void release_to_async_completion() noexcept
  {
    active_ = false;
  }

private:
  SceneLeaseRegistry & registry_;
  std::string ticket_;
  bool active_{true};
};

}  // namespace manipulation::scene

#endif  // SCENE__OPERATION_COMPLETION_GUARD_HPP_
