// Responsibility: Owns startup-selected transfer strategies and enforces the production registry.

#ifndef MANIPULATION__WORKFLOW__STRATEGY_REGISTRY_HPP_
#define MANIPULATION__WORKFLOW__STRATEGY_REGISTRY_HPP_

#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "manipulation/workflow/transfer_workflow_ports.hpp"

namespace manipulation::workflow
{

inline constexpr std::string_view kPickLiftPlaceStrategyId{"pick_lift_place"};

/** Small owning registry with an explicit production/test construction split. */
class StrategyRegistry final
{
public:
  /**
   * Build the production registry.
   *
   * The supplied strategy must report exactly `pick_lift_place`; no second
   * production ID is accepted by this API.
   */
  [[nodiscard]] static StrategyRegistry production(
    std::unique_ptr<TransferExecutionStrategy> pick_lift_place);

  /** Inject arbitrary compiled strategies only for isolated composition tests. */
  [[nodiscard]] static StrategyRegistry for_testing(
    std::vector<std::unique_ptr<TransferExecutionStrategy>> strategies);

  StrategyRegistry(const StrategyRegistry &) = delete;
  StrategyRegistry & operator=(const StrategyRegistry &) = delete;
  StrategyRegistry(StrategyRegistry &&) noexcept = default;
  StrategyRegistry & operator=(StrategyRegistry &&) noexcept = default;
  ~StrategyRegistry() = default;

  /** Throw `std::invalid_argument` for an unknown startup strategy ID. */
  [[nodiscard]] TransferExecutionStrategy & require(std::string_view strategy_id);
  [[nodiscard]] const TransferExecutionStrategy & require(
    std::string_view strategy_id) const;

  [[nodiscard]] std::vector<std::string> available_ids() const;
  [[nodiscard]] std::size_t size() const noexcept;

private:
  explicit StrategyRegistry(
    std::map<std::string, std::unique_ptr<TransferExecutionStrategy>> strategies);

  std::map<std::string, std::unique_ptr<TransferExecutionStrategy>> strategies_;
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__STRATEGY_REGISTRY_HPP_
